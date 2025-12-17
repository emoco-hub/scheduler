# ros2-scheduler

ROS2 scheduler node for controlling metering socket relays based on time, sun position, and relay state conditions.

## Features

- **Time-based scheduling**: Trigger at specific times with day-of-week patterns
- **Sunrise/sunset triggers**: Control relays relative to sun position (requires `astral` package)
- **Relay state conditions**: Use one relay as a master switch to control others
- **YAML configuration**: Load schedules from config files
- **Live parameter updates**: Modify schedules at runtime via ROS parameters
- **Persistent configuration**: Save/reload configuration via services

## Metering Socket Relay Interface

The scheduler works with metering socket relay nodes that expose:

**Publishing (relay state):**
- `/<namespace>/closed` - `std_msgs/Bool` - Relay on/off state
- `/<namespace>/connected` - `std_msgs/Bool` - Connection status
- `/<namespace>/energy` - `std_msgs/Float32` - Energy reading (kWh)
- `/<namespace>/power` - `std_msgs/Float32` - Power reading (W)
- `/<namespace>/rssi` - `std_msgs/Int32` - Signal strength

**Subscribing (relay control):**
- `/<namespace>/set_closed` - `std_msgs/Bool` - Command to set relay state

**Services:**
- `/<namespace>/publish_state` - `std_srvs/Trigger` - Request state publish

## Installation

```bash
# Clone the repository
git clone <repository-url> scheduler
cd scheduler

# Install Python dependencies
sudo apt update
sudo apt install -y python3-yaml python3-astral

# Build the package
colcon build
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Run with a config file
ros2 run scheduler scheduler_node --ros-args \
  -p config_file:=/path/to/schedule.yaml

# Run with parameters
ros2 run scheduler scheduler_node --ros-args \
  -p latitude:=59.3293 \
  -p longitude:=18.0686 \
  -p timezone:=Europe/Stockholm
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_file` | string | "" | Path to YAML configuration file |
| `latitude` | float | 59.3293 | Location latitude for sun calculations |
| `longitude` | float | 18.0686 | Location longitude for sun calculations |
| `timezone` | string | "Europe/Stockholm" | Timezone for schedule calculations |
| `check_interval_sec` | float | 1.0 | Interval for checking schedules |
| `schedules_yaml` | string | "" | Schedule rules as YAML string (for live updates) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/save_config` | `std_srvs/Trigger` | Save current config to file |
| `~/reload_config` | `std_srvs/Trigger` | Reload config from file |
| `~/all_lights_on` | `std_srvs/Trigger` | Turn ON all slave relays |
| `~/all_lights_off` | `std_srvs/Trigger` | Turn OFF all slave relays |
| `~/sync_to_master` | `std_srvs/Trigger` | Sync slaves to master's current state |

### Trigger Services for Studio Panels

The scheduler provides trigger services that can be used with studio panels:

```bash
# Turn on all lights
ros2 service call /scheduler/all_lights_on std_srvs/srv/Trigger

# Turn off all lights
ros2 service call /scheduler/all_lights_off std_srvs/srv/Trigger

# Sync slaves to master's current state (useful after startup)
ros2 service call /scheduler/sync_to_master std_srvs/srv/Trigger
```

These use the `std_srvs/Trigger` type which is compatible with studio panel buttons.

### Live Configuration Updates

Update schedules at runtime via parameters:

```bash
ros2 param set /scheduler schedules_yaml "
- name: Quick test
  trigger: time
  time: '14:30'
  relays: [/livingroom/lamp]
  action: close
"
```

### Save/Reload Configuration

```bash
# Save current configuration to file
ros2 service call /scheduler/save_config std_srvs/srv/Trigger

# Reload configuration from file
ros2 service call /scheduler/reload_config std_srvs/srv/Trigger
```

## Configuration File Format

See [config/example_schedule.yaml](config/example_schedule.yaml) for a complete example.

### Basic Structure

```yaml
# Location for sunrise/sunset calculations
location:
  latitude: 59.3293
  longitude: 18.0686
  timezone: Europe/Stockholm

# Relays to monitor (optional - auto-discovered from schedules)
relays:
  - /lab/relay1
  - /lab/relay2

# Schedule rules
schedules:
  - name: "Rule name"
    trigger: time|sunrise|sunset|follow
    # ... trigger-specific options
    relays:
      - /lab/relay1
    action: close|open
    enabled: true
```

### Time-Based Schedule

```yaml
- name: "Morning on"
  trigger: time
  time: "07:00"
  days: [0, 1, 2, 3, 4]  # Mon-Fri (0=Monday, 6=Sunday)
  relays: [/livingroom/lamp]
  action: close
```

### Sunrise/Sunset Schedule

```yaml
- name: "Sunset lights"
  trigger: sunset
  offset_minutes: -30  # 30 min before sunset
  days: [0, 1, 2, 3, 4, 5, 6]  # Every day
  relays: [/livingroom/lamp]
  action: close
```

### Follow Schedule (Master/Slave)

Have relays mirror another relay's state:

```yaml
- name: "Master controls all lights"
  trigger: follow
  source_relay: /hallway/main_switch
  relays: [/livingroom/lamp, /kitchen/lights, /porch/lights]
```

Slaves automatically mirror the source - when source closes, slaves close; when source opens, slaves open.

## Hardcoded Examples

The scheduler includes working hardcoded examples that demonstrate programmatic relay control:

### Master/Slave Relay Control

The node is configured with a master relay that controls multiple slave relays. When the master relay state changes, all slaves follow automatically:

```python
# Configured in __init__:
self.master_relay = '/hallway/main_switch'
self.slave_relays = ['/livingroom/lamp', '/kitchen/lights', '/porch/lights']
```

When `/hallway/main_switch` closes, all three slave relays will also close. When it opens, they all open.

To customize, edit the `master_relay` and `slave_relays` variables in `scheduler_node.py`.

### Trigger Services

Two trigger services demonstrate group control:

- `~/all_lights_on` - Closes all slave relays
- `~/all_lights_off` - Opens all slave relays

These can be connected to studio panel buttons for one-touch control.

## Additional Code Snippets

The scheduler node source includes commented code snippets demonstrating:

1. Manual relay control via publisher
2. Subscribing to monitor relay state
3. Calling the publish_state service
4. Custom time-based scheduling logic
5. Power-based automation (threshold triggers)

See the bottom of [scheduler/scheduler_node.py](scheduler/scheduler_node.py) for these examples.

## Development

### Project Structure

```
scheduler/
├── config/
│   └── example_schedule.yaml   # Example configuration
├── resource/
│   └── scheduler               # Package marker
├── scheduler/
│   ├── __init__.py
│   └── scheduler_node.py       # Main node implementation
├── package.xml                 # ROS2 package manifest
├── setup.cfg                   # Python setup config
├── setup.py                    # Python package setup
└── README.md
```

### Extending the Scheduler

To add custom trigger types or actions:

1. Add new `TriggerType` enum value
2. Implement parsing in `_parse_single_schedule()`
3. Add check logic in `_check_schedules()` or create a new check method
4. Update `_generate_config_dict()` for save/reload support

## License

MIT License - See [LICENSE](LICENSE) for details.
