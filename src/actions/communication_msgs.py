# This file contains fake Action Messages for offline mode


class FakeGoal:
    def __init__(self, object: str, location: str):
        self.object = object
        self.location = location


class MoveToGoal(FakeGoal):
    def __init__(self, object: str, location: str):
        super().__init__(object, location)


class PickupObjectGoal(FakeGoal):
    def __init__(self, object: str, location: str):
        super().__init__(object, location)


class PutObjectGoal(FakeGoal):
    def __init__(self, object: str, location: str):
        super().__init__(object, location)
