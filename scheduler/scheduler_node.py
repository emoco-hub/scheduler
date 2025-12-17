#!/usr/bin/env python3
"""
Scheduler Node - Controls metering socket relays based on schedules and conditions.

This node supports:
- Time-based scheduling (specific times, day-of-week patterns)
- Sun position triggers (sunrise/sunset with offsets)
- Relay state conditions (one relay controls others)
- YAML configuration with live parameter updates
- Persistent configuration saving

Metering Socket Relay Interface:
  PUBLISHING:
    /<namespace>/closed        - std_msgs/msg/Bool (relay on/off state)
    /<namespace>/connected     - std_msgs/msg/Bool (connection status)
    /<namespace>/energy        - std_msgs/msg/Float32 (energy reading)
    /<namespace>/power         - std_msgs/msg/Float32 (power reading)
    /<namespace>/rssi          - std_msgs/msg/Int32 (signal strength)

  SUBSCRIBING:
    /<namespace>/set_closed    - std_msgs/msg/Bool (command to set relay state)
    /reset                     - std_msgs/msg/Bool (reset command)
    /restart                   - std_msgs/msg/String (restart command)

  SERVICES:
    /<namespace>/get_type_description - type_description_interfaces/srv/GetTypeDescription
    /<namespace>/publish_state        - std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger

import yaml
import os
from datetime import datetime, time, timedelta, timezone
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import threading

# Optional: astral for sunrise/sunset calculations
# Install with: sudo apt install python3-astral
try:
    from astral import LocationInfo
    from astral.sun import sun
    import zoneinfo
    ASTRAL_AVAILABLE = True
except ImportError:
    ASTRAL_AVAILABLE = False
    print("Warning: astral package not installed. Sunrise/sunset triggers disabled.")
    print("Install with: pip install astral")


class TriggerType(Enum):
    """Types of schedule triggers."""
    TIME = "time"           # Specific time of day
    SUNRISE = "sunrise"     # Relative to sunrise
    SUNSET = "sunset"       # Relative to sunset
    FOLLOW = "follow"       # Slaves mirror master relay state


@dataclass
class RelayState:
    """Tracks the current state of a relay."""
    namespace: str
    closed: bool = False
    connected: bool = False
    energy: float = 0.0
    power: float = 0.0
    rssi: int = 0
    last_update: Optional[datetime] = None


@dataclass
class ScheduleRule:
    """A single scheduling rule."""
    name: str
    trigger_type: TriggerType
    target_relays: List[str]  # Relay namespaces to control
    action: bool              # True = close relay, False = open relay

    # Time-based trigger options
    trigger_time: Optional[time] = None
    days_of_week: List[int] = field(default_factory=lambda: list(range(7)))  # 0=Mon, 6=Sun

    # Sun position trigger options
    sun_offset_minutes: int = 0  # Minutes before(-) or after(+) sunrise/sunset

    # Follow trigger options - slaves mirror source relay state
    source_relay: Optional[str] = None

    enabled: bool = True


class SchedulerNode(Node):
    """
    ROS2 Node that schedules relay control based on various triggers.

    Parameters:
        config_file (str): Path to YAML configuration file
        latitude (float): Location latitude for sun calculations
        longitude (float): Location longitude for sun calculations
        timezone (str): Timezone name (e.g., 'Europe/Stockholm')
        check_interval_sec (float): How often to check schedules (default: 1.0)
    """

    def __init__(self):
        super().__init__('scheduler')

        # Declare parameters
        self._declare_parameters()

        # State tracking
        self.relay_states: Dict[str, RelayState] = {}
        self.schedule_rules: List[ScheduleRule] = []
        self.relay_publishers: Dict[str, Any] = {}  # namespace -> publisher
        self.relay_subscribers: Dict[str, Dict[str, Any]] = {}  # namespace -> {topic: subscriber}

        # Lock for thread safety
        self._lock = threading.Lock()

        # Load configuration
        self._load_config()

        # Set up parameter callback for live updates
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Timer for schedule checking
        check_interval = self.get_parameter('check_interval_sec').value
        self.schedule_timer = self.create_timer(check_interval, self._check_schedules)

        # Service for dumping current config to file
        self.save_config_srv = self.create_service(
            Trigger,
            '~/save_config',
            self._save_config_callback
        )

        # Service for reloading config from file
        self.reload_config_srv = self.create_service(
            Trigger,
            '~/reload_config',
            self._reload_config_callback
        )

        # =====================================================================
        # EXAMPLE: Trigger service for group control
        # =====================================================================
        # This service allows external control of multiple relays via a single
        # trigger call. Useful for integration with studio panels that support
        # trigger topics/services.
        #
        # Call with: ros2 service call /scheduler/all_lights_on std_srvs/srv/Trigger
        # Call with: ros2 service call /scheduler/all_lights_off std_srvs/srv/Trigger
        # ---------------------------------------------------------------------
        self.all_lights_on_srv = self.create_service(
            Trigger,
            '~/all_lights_on',
            self._all_lights_on_callback
        )
        self.all_lights_off_srv = self.create_service(
            Trigger,
            '~/all_lights_off',
            self._all_lights_off_callback
        )

        # =====================================================================
        # EXAMPLE: Hardcoded master/slave relay control
        # =====================================================================
        # This demonstrates programmatic relay control where one relay acts as
        # a master switch. When the master relay state changes, slave relays
        # follow automatically.
        #
        # Configure master and slave relays here:
        # ---------------------------------------------------------------------
        self.master_relay = '/hallway/main_switch'
        self.slave_relays = ['/livingroom/lamp', '/kitchen/lights', '/porch/lights']

        # Set up the master relay (this subscribes to its state)
        self._setup_relay(self.master_relay)

        # Set up all slave relays
        for slave in self.slave_relays:
            self._setup_relay(slave)

        # Service to sync slaves to master's current state
        # Call this after startup to initialize slaves from master
        self.sync_to_master_srv = self.create_service(
            Trigger,
            '~/sync_to_master',
            self._sync_to_master_callback
        )

        self.get_logger().info('Scheduler node initialized')
        self.get_logger().info(f'Master relay: {self.master_relay}')
        self.get_logger().info(f'Slave relays: {self.slave_relays}')

    def _declare_parameters(self):
        """Declare all node parameters."""
        self.declare_parameter(
            'config_file',
            '',
            ParameterDescriptor(description='Path to YAML configuration file')
        )
        self.declare_parameter(
            'latitude',
            59.3293,  # Stockholm default
            ParameterDescriptor(description='Location latitude for sun calculations')
        )
        self.declare_parameter(
            'longitude',
            18.0686,  # Stockholm default
            ParameterDescriptor(description='Location longitude for sun calculations')
        )
        self.declare_parameter(
            'timezone',
            'Europe/Stockholm',
            ParameterDescriptor(description='Timezone for schedule calculations')
        )
        self.declare_parameter(
            'check_interval_sec',
            1.0,
            ParameterDescriptor(description='Interval in seconds for checking schedules')
        )
        # Dynamic parameter for schedule rules (as YAML string)
        self.declare_parameter(
            'schedules_yaml',
            '',
            ParameterDescriptor(description='Schedule rules as YAML string (for live updates)')
        )

    def _on_parameter_change(self, params: List[Parameter]) -> SetParametersResult:
        """Handle parameter changes at runtime."""
        for param in params:
            if param.name == 'schedules_yaml' and param.value:
                try:
                    self._parse_schedules_yaml(param.value)
                    self.get_logger().info('Schedules updated from parameter')
                except Exception as e:
                    self.get_logger().error(f'Failed to parse schedules_yaml: {e}')
                    return SetParametersResult(successful=False, reason=str(e))
            elif param.name == 'config_file' and param.value:
                self._load_config()
        return SetParametersResult(successful=True)

    def _load_config(self):
        """Load configuration from YAML file."""
        config_file = self.get_parameter('config_file').value
        if not config_file or not os.path.exists(config_file):
            self.get_logger().warn(f'Config file not found: {config_file}')
            return

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            # Update location parameters if specified in config
            if 'location' in config:
                loc = config['location']
                if 'latitude' in loc:
                    self.set_parameters([Parameter('latitude', Parameter.Type.DOUBLE, loc['latitude'])])
                if 'longitude' in loc:
                    self.set_parameters([Parameter('longitude', Parameter.Type.DOUBLE, loc['longitude'])])
                if 'timezone' in loc:
                    self.set_parameters([Parameter('timezone', Parameter.Type.STRING, loc['timezone'])])

            # Parse relay configurations
            if 'relays' in config:
                for relay_ns in config['relays']:
                    self._setup_relay(relay_ns)

            # Parse schedule rules
            if 'schedules' in config:
                self._parse_schedules(config['schedules'])

            self.get_logger().info(f'Loaded config from {config_file}')

        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')

    def _parse_schedules(self, schedules_config: List[Dict]):
        """Parse schedule rules from config dictionary."""
        with self._lock:
            self.schedule_rules.clear()

            for sched in schedules_config:
                try:
                    rule = self._parse_single_schedule(sched)
                    if rule:
                        self.schedule_rules.append(rule)
                        self.get_logger().info(f'Added schedule rule: {rule.name}')
                except Exception as e:
                    self.get_logger().error(f'Failed to parse schedule: {e}')

    def _parse_schedules_yaml(self, yaml_str: str):
        """Parse schedules from YAML string parameter."""
        schedules_config = yaml.safe_load(yaml_str)
        if isinstance(schedules_config, list):
            self._parse_schedules(schedules_config)

    def _parse_single_schedule(self, sched: Dict) -> Optional[ScheduleRule]:
        """Parse a single schedule rule from config."""
        name = sched.get('name', 'unnamed')
        trigger_type_str = sched.get('trigger', 'time')
        trigger_type = TriggerType(trigger_type_str)

        target_relays = sched.get('relays', [])
        if isinstance(target_relays, str):
            target_relays = [target_relays]

        action = sched.get('action', 'close') == 'close'

        rule = ScheduleRule(
            name=name,
            trigger_type=trigger_type,
            target_relays=target_relays,
            action=action,
            enabled=sched.get('enabled', True)
        )

        # Parse trigger-specific options
        if trigger_type == TriggerType.TIME:
            time_str = sched.get('time', '00:00')
            hour, minute = map(int, time_str.split(':'))
            rule.trigger_time = time(hour, minute)
            rule.days_of_week = sched.get('days', list(range(7)))

        elif trigger_type in (TriggerType.SUNRISE, TriggerType.SUNSET):
            rule.sun_offset_minutes = sched.get('offset_minutes', 0)
            rule.days_of_week = sched.get('days', list(range(7)))

        elif trigger_type == TriggerType.FOLLOW:
            rule.source_relay = sched.get('source_relay')
            # Note: action field is ignored for FOLLOW - slaves mirror source state

        # Ensure relays are set up
        for relay_ns in target_relays:
            self._setup_relay(relay_ns)
        if rule.source_relay:
            self._setup_relay(rule.source_relay)

        return rule

    def _setup_relay(self, namespace: str):
        """Set up publishers and subscribers for a relay."""
        if namespace in self.relay_states:
            return  # Already set up

        self.get_logger().info(f'Setting up relay: {namespace}')

        # Initialize state tracking
        self.relay_states[namespace] = RelayState(namespace=namespace)

        # Create publisher for controlling the relay
        # -------------------------------------------------------------------------
        # RELAY CONTROL: Publish to /<namespace>/set_closed to control the relay
        # -------------------------------------------------------------------------
        self.relay_publishers[namespace] = self.create_publisher(
            Bool,
            f'{namespace}/set_closed',
            10
        )

        # Subscribe to relay state topics
        self.relay_subscribers[namespace] = {}

        # -------------------------------------------------------------------------
        # RELAY STATE: Subscribe to /<namespace>/closed to monitor relay state
        # -------------------------------------------------------------------------
        self.relay_subscribers[namespace]['closed'] = self.create_subscription(
            Bool,
            f'{namespace}/closed',
            lambda msg, ns=namespace: self._on_relay_closed(ns, msg),
            10
        )

        # -------------------------------------------------------------------------
        # RELAY CONNECTION: Subscribe to /<namespace>/connected for connection status
        # -------------------------------------------------------------------------
        self.relay_subscribers[namespace]['connected'] = self.create_subscription(
            Bool,
            f'{namespace}/connected',
            lambda msg, ns=namespace: self._on_relay_connected(ns, msg),
            10
        )

        # -------------------------------------------------------------------------
        # RELAY POWER: Subscribe to /<namespace>/power for power readings
        # -------------------------------------------------------------------------
        self.relay_subscribers[namespace]['power'] = self.create_subscription(
            Float32,
            f'{namespace}/power',
            lambda msg, ns=namespace: self._on_relay_power(ns, msg),
            10
        )

        # -------------------------------------------------------------------------
        # RELAY ENERGY: Subscribe to /<namespace>/energy for energy readings
        # -------------------------------------------------------------------------
        self.relay_subscribers[namespace]['energy'] = self.create_subscription(
            Float32,
            f'{namespace}/energy',
            lambda msg, ns=namespace: self._on_relay_energy(ns, msg),
            10
        )

    # -------------------------------------------------------------------------
    # RELAY STATE CALLBACKS
    # These callbacks are triggered when relay state messages are received
    # -------------------------------------------------------------------------

    def _on_relay_closed(self, namespace: str, msg: Bool):
        """Handle relay closed state update."""
        with self._lock:
            if namespace in self.relay_states:
                old_state = self.relay_states[namespace].closed
                self.relay_states[namespace].closed = msg.data
                self.relay_states[namespace].last_update = datetime.now()

                # Check if this triggers any follow rules (from config)
                if old_state != msg.data:
                    self._check_follow_triggers(namespace, msg.data)

                    # =============================================================
                    # EXAMPLE: Hardcoded master/slave control
                    # =============================================================
                    # When the master relay changes state, all slave relays follow.
                    # This is the programmatic equivalent of the YAML relay trigger.
                    # -------------------------------------------------------------
                    if namespace == self.master_relay:
                        self.get_logger().info(
                            f'Master relay {namespace} changed to '
                            f'{"CLOSED" if msg.data else "OPEN"}, '
                            f'updating {len(self.slave_relays)} slave relays'
                        )
                        for slave in self.slave_relays:
                            self._set_relay_state(slave, msg.data)

    def _on_relay_connected(self, namespace: str, msg: Bool):
        """Handle relay connection state update."""
        with self._lock:
            if namespace in self.relay_states:
                self.relay_states[namespace].connected = msg.data

    def _on_relay_power(self, namespace: str, msg: Float32):
        """Handle relay power reading update."""
        with self._lock:
            if namespace in self.relay_states:
                self.relay_states[namespace].power = msg.data

    def _on_relay_energy(self, namespace: str, msg: Float32):
        """Handle relay energy reading update."""
        with self._lock:
            if namespace in self.relay_states:
                self.relay_states[namespace].energy = msg.data

    # -------------------------------------------------------------------------
    # SCHEDULE CHECKING
    # -------------------------------------------------------------------------

    def _check_schedules(self):
        """Timer callback to check all time-based schedules."""
        now = datetime.now()
        current_time = now.time()
        current_day = now.weekday()

        with self._lock:
            for rule in self.schedule_rules:
                if not rule.enabled:
                    continue

                if rule.trigger_type == TriggerType.TIME:
                    self._check_time_trigger(rule, current_time, current_day)

                elif rule.trigger_type == TriggerType.SUNRISE:
                    self._check_sun_trigger(rule, now, is_sunrise=True)

                elif rule.trigger_type == TriggerType.SUNSET:
                    self._check_sun_trigger(rule, now, is_sunrise=False)

    def _check_time_trigger(self, rule: ScheduleRule, current_time: time, current_day: int):
        """Check if a time-based rule should trigger."""
        if current_day not in rule.days_of_week:
            return

        if rule.trigger_time is None:
            return

        # Check if we're within 1 second of the trigger time
        trigger_dt = datetime.combine(datetime.today(), rule.trigger_time)
        current_dt = datetime.combine(datetime.today(), current_time)
        diff = abs((trigger_dt - current_dt).total_seconds())

        if diff < 1.0:
            self._execute_rule(rule)

    def _check_sun_trigger(self, rule: ScheduleRule, now: datetime, is_sunrise: bool):
        """Check if a sun-position-based rule should trigger."""
        if not ASTRAL_AVAILABLE:
            return

        current_day = now.weekday()
        if current_day not in rule.days_of_week:
            return

        # Get sun times for today
        try:
            lat = self.get_parameter('latitude').value
            lon = self.get_parameter('longitude').value
            tz_name = self.get_parameter('timezone').value

            # Get timezone-aware current time
            tz = zoneinfo.ZoneInfo(tz_name)
            now_aware = datetime.now(tz)

            location = LocationInfo(timezone=tz_name, latitude=lat, longitude=lon)
            s = sun(location.observer, date=now_aware.date(), tzinfo=tz)

            if is_sunrise:
                sun_time = s['sunrise']
            else:
                sun_time = s['sunset']

            # Apply offset
            trigger_time = sun_time + timedelta(minutes=rule.sun_offset_minutes)

            # Check if we're within 1 second of the trigger time
            diff = abs((trigger_time - now_aware).total_seconds())
            if diff < 1.0:
                self._execute_rule(rule)

        except Exception as e:
            self.get_logger().error(f'Sun calculation error: {e}')

    def _check_follow_triggers(self, source_namespace: str, new_state: bool):
        """Check if any follow rules should trigger when source relay changes."""
        for rule in self.schedule_rules:
            if not rule.enabled:
                continue
            if rule.trigger_type != TriggerType.FOLLOW:
                continue
            if rule.source_relay != source_namespace:
                continue

            self.get_logger().info(
                f'Follow rule "{rule.name}": '
                f'source {source_namespace} -> {"closed" if new_state else "open"}, '
                f'mirroring to {rule.target_relays}'
            )
            for relay_ns in rule.target_relays:
                self._set_relay_state(relay_ns, new_state)

    # -------------------------------------------------------------------------
    # RELAY CONTROL
    # -------------------------------------------------------------------------

    def _execute_rule(self, rule: ScheduleRule):
        """Execute a schedule rule - control the target relays."""
        self.get_logger().info(
            f'Executing rule "{rule.name}": '
            f'{"closing" if rule.action else "opening"} {rule.target_relays}'
        )

        for relay_ns in rule.target_relays:
            self._set_relay_state(relay_ns, rule.action)

    def _set_relay_state(self, namespace: str, closed: bool):
        """
        Set the state of a relay.

        -------------------------------------------------------------------------
        EXAMPLE: Publishing to control a relay
        -------------------------------------------------------------------------
        To control a relay, publish a Bool message to /<namespace>/set_closed:

            msg = Bool()
            msg.data = True   # Close the relay (turn ON)
            # msg.data = False  # Open the relay (turn OFF)
            publisher.publish(msg)

        -------------------------------------------------------------------------
        """
        if namespace not in self.relay_publishers:
            self.get_logger().warn(f'No publisher for relay: {namespace}')
            return

        msg = Bool()
        msg.data = closed
        self.relay_publishers[namespace].publish(msg)

        self.get_logger().debug(
            f'Set relay {namespace} to {"closed" if closed else "open"}'
        )

    # -------------------------------------------------------------------------
    # SERVICES FOR INTERACTIVE USE
    # -------------------------------------------------------------------------

    def _save_config_callback(self, request, response):
        """Service callback to save current configuration to file."""
        config_file = self.get_parameter('config_file').value
        if not config_file:
            response.success = False
            response.message = 'No config_file parameter set'
            return response

        try:
            config = self._generate_config_dict()
            with open(config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

            response.success = True
            response.message = f'Configuration saved to {config_file}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save config: {e}'
            self.get_logger().error(response.message)

        return response

    def _reload_config_callback(self, request, response):
        """Service callback to reload configuration from file."""
        try:
            self._load_config()
            response.success = True
            response.message = 'Configuration reloaded'
        except Exception as e:
            response.success = False
            response.message = f'Failed to reload config: {e}'
        return response

    # -------------------------------------------------------------------------
    # EXAMPLE: Trigger services for group control
    # -------------------------------------------------------------------------
    # These services provide a simple way to control multiple relays at once.
    # They use the Trigger service type which is compatible with studio panels.
    #
    # To call from command line:
    #   ros2 service call /scheduler/all_lights_on std_srvs/srv/Trigger
    #   ros2 service call /scheduler/all_lights_off std_srvs/srv/Trigger
    #
    # To create additional trigger services, copy this pattern and modify
    # the relay list and action as needed.
    # -------------------------------------------------------------------------

    def _all_lights_on_callback(self, request, response):
        """
        Trigger service to turn ON all configured slave relays.

        This demonstrates how to create a trigger-based control point that
        can be called from studio panels or other ROS2 nodes.
        """
        self.get_logger().info('Trigger: all_lights_on called')

        # Control all slave relays (you can customize this list)
        relays_to_control = self.slave_relays

        for relay in relays_to_control:
            self._set_relay_state(relay, closed=True)

        response.success = True
        response.message = f'Turned ON {len(relays_to_control)} relays: {relays_to_control}'
        return response

    def _all_lights_off_callback(self, request, response):
        """
        Trigger service to turn OFF all configured slave relays.

        This demonstrates how to create a trigger-based control point that
        can be called from studio panels or other ROS2 nodes.
        """
        self.get_logger().info('Trigger: all_lights_off called')

        # Control all slave relays (you can customize this list)
        relays_to_control = self.slave_relays

        for relay in relays_to_control:
            self._set_relay_state(relay, closed=False)

        response.success = True
        response.message = f'Turned OFF {len(relays_to_control)} relays: {relays_to_control}'
        return response

    def _sync_to_master_callback(self, request, response):
        """
        Trigger service to sync all slave relays to the master's current state.

        Call this after startup to initialize slaves based on whatever state
        the master relay currently has. This is useful when:
        - The scheduler node restarts and slaves should match master
        - You want to manually force slaves to sync with master

        Call with: ros2 service call /scheduler/sync_to_master std_srvs/srv/Trigger
        """
        self.get_logger().info('Trigger: sync_to_master called')

        # Get current master state
        master_state = self.relay_states.get(self.master_relay)
        if master_state is None:
            response.success = False
            response.message = f'Master relay {self.master_relay} not found'
            return response

        current_state = master_state.closed
        self.get_logger().info(
            f'Syncing {len(self.slave_relays)} slaves to master state: '
            f'{"CLOSED" if current_state else "OPEN"}'
        )

        # Set all slaves to match master
        for slave in self.slave_relays:
            self._set_relay_state(slave, current_state)

        response.success = True
        response.message = (
            f'Synced {len(self.slave_relays)} relays to master state '
            f'({"closed" if current_state else "open"}): {self.slave_relays}'
        )
        return response

    def _generate_config_dict(self) -> Dict:
        """Generate configuration dictionary from current state."""
        config = {
            'location': {
                'latitude': self.get_parameter('latitude').value,
                'longitude': self.get_parameter('longitude').value,
                'timezone': self.get_parameter('timezone').value,
            },
            'relays': list(self.relay_states.keys()),
            'schedules': []
        }

        for rule in self.schedule_rules:
            sched = {
                'name': rule.name,
                'trigger': rule.trigger_type.value,
                'relays': rule.target_relays,
                'action': 'close' if rule.action else 'open',
                'enabled': rule.enabled,
            }

            if rule.trigger_type == TriggerType.TIME:
                sched['time'] = rule.trigger_time.strftime('%H:%M') if rule.trigger_time else '00:00'
                sched['days'] = rule.days_of_week
            elif rule.trigger_type in (TriggerType.SUNRISE, TriggerType.SUNSET):
                sched['offset_minutes'] = rule.sun_offset_minutes
                sched['days'] = rule.days_of_week
            elif rule.trigger_type == TriggerType.FOLLOW:
                sched['source_relay'] = rule.source_relay
                # Remove action field for FOLLOW type as it's not used
                del sched['action']

            config['schedules'].append(sched)

        return config


# -------------------------------------------------------------------------
# EXAMPLE CODE SNIPPETS FOR INTERACTING WITH RELAY NODES
# -------------------------------------------------------------------------

"""
# =========================================================================
# SNIPPET 1: Manually publish to control a relay
# =========================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RelayController(Node):
    def __init__(self):
        super().__init__('relay_controller')

        # Create publisher for the relay's set_closed topic
        self.relay_pub = self.create_publisher(
            Bool,
            '/livingroom/lamp/set_closed',
            10
        )

    def close_relay(self):
        '''Close (turn ON) the relay.'''
        msg = Bool()
        msg.data = True
        self.relay_pub.publish(msg)
        self.get_logger().info('Relay closed')

    def open_relay(self):
        '''Open (turn OFF) the relay.'''
        msg = Bool()
        msg.data = False
        self.relay_pub.publish(msg)
        self.get_logger().info('Relay opened')


# =========================================================================
# SNIPPET 2: Subscribe to monitor relay state
# =========================================================================

class RelayMonitor(Node):
    def __init__(self):
        super().__init__('relay_monitor')

        # Subscribe to relay state topics
        self.closed_sub = self.create_subscription(
            Bool,
            '/livingroom/lamp/closed',
            self.on_closed,
            10
        )
        self.power_sub = self.create_subscription(
            Float32,
            '/livingroom/lamp/power',
            self.on_power,
            10
        )

    def on_closed(self, msg):
        self.get_logger().info(f'Relay state: {"CLOSED" if msg.data else "OPEN"}')

    def on_power(self, msg):
        self.get_logger().info(f'Power: {msg.data:.2f}W')


# =========================================================================
# SNIPPET 3: Call the publish_state service to request state update
# =========================================================================

from std_srvs.srv import Trigger

class RelayServiceClient(Node):
    def __init__(self):
        super().__init__('relay_service_client')

        self.publish_state_client = self.create_client(
            Trigger,
            '/livingroom/lamp/publish_state'
        )

    def request_state_publish(self):
        '''Request the relay to publish its current state.'''
        if not self.publish_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available')
            return

        request = Trigger.Request()
        future = self.publish_state_client.call_async(request)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        response = future.result()
        self.get_logger().info(f'Service response: {response.message}')


# =========================================================================
# SNIPPET 4: Time-based relay control with custom logic
# =========================================================================

from datetime import datetime, time

class CustomScheduler(Node):
    def __init__(self):
        super().__init__('custom_scheduler')

        # Publisher for relay control
        self.relay_pub = self.create_publisher(
            Bool,
            '/livingroom/lamp/set_closed',
            10
        )

        # Timer to check every minute
        self.timer = self.create_timer(60.0, self.check_schedule)

        # Define schedule: {(hour, minute): desired_state}
        self.schedule = {
            (7, 0): True,   # Turn ON at 7:00 AM
            (22, 0): False, # Turn OFF at 10:00 PM
        }

    def check_schedule(self):
        now = datetime.now()
        current_key = (now.hour, now.minute)

        if current_key in self.schedule:
            state = self.schedule[current_key]
            msg = Bool()
            msg.data = state
            self.relay_pub.publish(msg)
            self.get_logger().info(
                f'Schedule triggered: {"closing" if state else "opening"} relay'
            )


# =========================================================================
# SNIPPET 5: Power-based automation (turn off if power exceeds threshold)
# =========================================================================

class PowerLimiter(Node):
    def __init__(self):
        super().__init__('power_limiter')

        self.power_threshold = 1000.0  # Watts

        self.power_sub = self.create_subscription(
            Float32,
            '/livingroom/lamp/power',
            self.on_power,
            10
        )

        self.relay_pub = self.create_publisher(
            Bool,
            '/livingroom/lamp/set_closed',
            10
        )

    def on_power(self, msg):
        if msg.data > self.power_threshold:
            self.get_logger().warn(
                f'Power {msg.data:.1f}W exceeds threshold {self.power_threshold}W, '
                'opening relay!'
            )
            shutdown_msg = Bool()
            shutdown_msg.data = False
            self.relay_pub.publish(shutdown_msg)
"""


def main(args=None):
    rclpy.init(args=args)

    node = SchedulerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
