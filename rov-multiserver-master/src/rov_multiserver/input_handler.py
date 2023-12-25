"""Pilot input handling module."""
from rov_event_bus.bus import *
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()


class _JoystickButton(object):
    """
    Helper class to represent the state of a button.

    It's used on _CommandGenerator class map the modifier buttons state.

    """

    def __init__(self):
        """
        Initialized the object.

        Creates the "__pushed" attribute (initial value: False).
        """
        self.__pushed = False

    def push(self):
        """
        Update the state as "pushed".

        Set the "__pushed" attribute to True.
        """
        self.__pushed = True

    def release(self):
        """
        Update the state "not pushed".

        Set the "__pushed" attribute to False (AKA: "unset" it).
        """
        self.__pushed = False

    def is_pushed(self):
        """
        Check if button is pushed.

        Returns
        -------
        bool
            True if pushed. False otherwise.

        """
        return self.__pushed is True

    def is_released(self):
        """
        Check if button is released.

        Returns
        -------
        bool
            True if released. False otherwise.

        """
        return self.__pushed is False


class _CommandGenerator(object):
    """Auxiliar class for converting input events into command events."""

    event_map = {
        # Movement commands
        'move-yaw-cmd': {
            'trigger_name': 'LXA', 'modifier_name': None, 'is_button': False},
        'move-surge-cmd': {
            'trigger_name': 'LYA', 'modifier_name': None, 'is_button': False},
        'pri-x-movement-cmd': {
            'trigger_name': 'RXA', 'modifier_name': None, 'is_button': False},
        'pri-y-movement-cmd': {
            'trigger_name': 'RYA', 'modifier_name': None, 'is_button': False},
        'sec-x-movement-cmd': {
            'trigger_name': 'XPD', 'modifier_name': None, 'is_button': False},
        'sec-y-movement-cmd': {
            'trigger_name': 'YPD', 'modifier_name': None, 'is_button': False},
        # Lights and camera servo commands
        'cycle-lights-cmd': {
            'trigger_name': 'STB', 'modifier_name': None, 'is_button': True},
        'cam-tilt-up-push': {
            'trigger_name': 'RBB', 'modifier_name': None, 'is_button': True,
            'on_push': True},
        'cam-tilt-up-release': {
            'trigger_name': 'RBB', 'modifier_name': None, 'is_button': True,
            'on_push': False},
        'cam-tilt-down-push': {
            'trigger_name': 'RTB', 'modifier_name': None, 'is_button': True,
            'on_push': True},
        'cam-tilt-down-release': {
            'trigger_name': 'RTB', 'modifier_name': None, 'is_button': True,
            'on_push': False},
        # Video registering commands
        'take-snapshot-cmd': {
            'trigger_name': 'BKB', 'modifier_name': None, 'is_button': True},
        # Navigation commands
        'toggle-stabilize-mode-cmd': {
            'trigger_name': 'YBT', 'modifier_name': None, 'is_button': True},
        'toggle-depth-hold-mode-cmd': {
            'trigger_name': 'XBT', 'modifier_name': None, 'is_button': True},
        'input-gain-up-cmd': {
            'trigger_name': 'LBB', 'modifier_name': None, 'is_button': True},
        'input-gain-down-cmd': {
            'trigger_name': 'LTB', 'modifier_name': None, 'is_button': True},
        'do-arm-cmd': {
            'trigger_name': 'ABT', 'modifier_name': None, 'is_button': True},
        'do-disarm-cmd': {
            'trigger_name': 'BBT', 'modifier_name': None, 'is_button': True},
        'arm-open-cmd': {
            'trigger_name': 'RSB', 'modifier_name': None, 'is_button': True,
            'on_push': True},
        'arm-close-cmd': {
            'trigger_name': 'LSB', 'modifier_name': None, 'is_button': True,
            'on_push': True},
        'arm-open-stop-cmd': {
            'trigger_name': 'RSB', 'modifier_name': None, 'is_button': True,
            'on_push': False},
        'arm-close-stop-cmd': {
            'trigger_name': 'LSB', 'modifier_name': None, 'is_button': True,
            'on_push': False}
    }

    def __init__(self):
        """
        Initializes the CommandGenerator object.

        Right now it just create the reference sets by calling the
        "_generate_reference_sets()" method.
        """
        self.__generate_reference_sets()

    def start(self):
        """
        Start the CommandGenerator object.

        Right now it just register the corresponding callback on the
        event bus.
        """
        event_bus.register_callback('input-event', self.on_input_event,
                                    URGENT, True)

    def stop(self):
        """
        Stop the CommandGenerator object.

        Right now it just unregister the previously registered callback
        on the event bus.
        """
        event_bus.unregister_callback('input-event', self.on_input_event)

    def __generate_reference_sets(self):
        """
        Create the "reference sets".

        These are based on the "event map":
        1.  **__trigger_set:** It stores the element name that should
            generate specific "command" events.
        2.  **__modifier_map:** It stores the objects that represent the
            modifier button states.
        """
        self.__trigger_set = set()
        self.__modifier_map = dict()
        for trigger_desc in self.event_map.values():
            self.__trigger_set.add(trigger_desc['trigger_name'])
            if trigger_desc['modifier_name'] is not None:
                self.__trigger_set.add(trigger_desc['modifier_name'])
                self.__modifier_map[trigger_desc['modifier_name']]\
                    = _JoystickButton()

    def on_input_event(self, event_description):
        """
        Callback method for every 'input-event'.

        This should work in the following way:
        1.  It should parse the event string, to obtain the element name
            and the intensity/state.
        2.  It should verify if the element name is on the trigger set.
        3.  If the input is a modifier, update its state on the mofifier
            map.
        4.  Verify if it should trigger an event. An event should be
            triggered if the event requires a modifier, and this
            modifier button it's pushed. Also, if the event is of type
            "button" its state should be on "R" (released) to trigger an
            event.
        5.  Trigger the event (just if it should), passing the intensity
            as a float when it isn't a "button event" (no arguments
            passed if it's a "button event").


        """
        trigger_name, intensity = event_description.split(';')
        if trigger_name in self.__trigger_set:
            # First check if input is a modifier and update its state
            if trigger_name in self.__modifier_map:
                if intensity == 'P':
                    self.__modifier_map[trigger_name].push()
                elif intensity == 'R':
                    self.__modifier_map[trigger_name].release()
            # Now check if an event should be triggered
            for event, trigger_desc in self.event_map.items():
                if trigger_name == trigger_desc['trigger_name']:
                    should_trigger = True
                    modifier = self.__modifier_map.get(
                        trigger_desc['modifier_name'])
                    on_push = trigger_desc.get('on_push')
                    if modifier is not None and modifier.is_released():
                        should_trigger = False
                    elif trigger_desc['is_button'] and on_push and \
                            intensity == 'R':
                        should_trigger = False
                    elif trigger_desc['is_button'] and not on_push and \
                            intensity == 'P':
                        should_trigger = False
                    if should_trigger:
                        if trigger_desc['is_button']:
                            event_bus.trigger(event)
                        else:
                            event_bus.trigger(event, float(intensity))


class InputHandler():
    """Module's main class."""

    def __init__(self):
        """
        Initializes the module.

        Creates every object used by the module:
        Right now it's just for _CommandGenerator (it's the only element
        on the module).
        """
        self._cmd_generator = _CommandGenerator()

    def start(self):
        """
        Starts module activity.

        Call the starting method for every object used by the module:
        Right now it's just for _CommandGenerator (it's the only element
        on the module).
        """
        self._cmd_generator.start()

    def stop(self):
        """
        Stops module activity.

        Call the stopping method for every object used by the module.
        Right now it's just for _CommandGenerator (it's the only element
        on the module).
        """
        self._cmd_generator.stop()
