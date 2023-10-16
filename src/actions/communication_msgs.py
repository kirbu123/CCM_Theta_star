# This file contains fake Action Messages for offline mode


class FakeGoal:
    def __init__(self, object: str = "unspecified", location: str = "unspecified"):
        self.object = object
        self.location = location


class MoveToGoal(FakeGoal):
    def __init__(self, object: str = "unspecified", location: str = "unspecified"):
        super().__init__(object, location)


class PickupObjectGoal(FakeGoal):
    def __init__(self, object: str = "unspecified", location: str = "unspecified"):
        super().__init__(object, location)


class PutObjectGoal(FakeGoal):
    def __init__(self, object: str = "unspecified", location: str = "unspecified"):
        super().__init__(object, location)


class FakeResult:
    def __init__(self, result: str = "success"):
        self.result = result


class MoveToResult(FakeResult):
    def __init__(self, result: str = "success"):
        super().__init__(result)


class PickupObjectResult(FakeResult):
    def __init__(self, result: str = "success"):
        super().__init__(result)


class PutObjectResult(FakeResult):
    def __init__(self, result: str = "success"):
        super().__init__(result)


class FakeFeedback:
    def __init__(self):
        pass


class MoveToFeedback(FakeFeedback):
    def __init__(self, x: float = 0.0, y: float = 0.0):
        super().__init__()
        self.x = x
        self.y = y
