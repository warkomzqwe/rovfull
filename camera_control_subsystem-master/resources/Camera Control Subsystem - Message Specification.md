# Message Specification for Camera Control Subsystem

*Puerto Montt, 3 de septiembre de 2019, Nicolás Hasbún A.*

## Topics

Topic involved in camera control:

- **/rov/camera/control_commands/**

Topic involved in ACK routine:

- **/rov/camera/control_commands/ack/**

## Format

Accepted Publisher messages in YAML format for **camera control**:

```yaml
- tilt_target: 0.0
- token_hex: ffffffff
```

```yaml
- pan_target: 0.0
- token_hex: ffffffff
```

```yaml
- tilt_speed: 0.0
- token_hex: ffffffff
```

```yaml
- pan_speed: 0.0
- token_hex: ffffffff
```

- Every **float value** must be between -1 and 1 value matching a real -100% to 100% physical capabilities.
- Token could be any **ASCII** compatible string, but it must be automatically generated as unique and random for every single message sent.

Listener message in YAML format for **ack routine**:

```yaml
- token_hex: ffffffff
```

- Listener sends token back as an ACK response.