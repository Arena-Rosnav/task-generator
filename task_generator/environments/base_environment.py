class BaseEnvironment:
    def __init__(self, namespace):
        pass

    def before_reset_task(self):
        raise NotImplementedError()

    def after_reset_task(self):
        raise NotImplementedError()

    def remove_all_obstacles(self):
        raise NotImplementedError()

    def spawn_random_dynamic_obstacle(self):
        raise NotImplementedError()

    def spawn_random_static_obstacles(self):
        raise NotImplementedError()

    def publish_goal(self, goal):
        raise NotImplementedError()

    def move_robot(self, pos):
        raise NotImplementedError()

    def spawn_robot(self):
        raise NotImplementedError()