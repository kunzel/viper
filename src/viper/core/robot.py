import logging; logger = logging.getLogger("viper." + __name__)

class Robot(object):

    def __init__(self, view_generator, view_controller, view_transition_model, view_evaluator, view_action):
        self._view_generator = view_generator
        self._view_controller = view_controller
        self._view_transition_model = view_transition_model
        self._view_evaluator = view_evaluator
        self._view_action = view_action
        self._current_view = None
        
    def set_current_view(self, view):
        self._current_view = view
        
    def get_current_view(self):
        return self._current_view

    def generate(self):
        return self._view_generator.generate()
    
    def cost(self, view1, view2):
        return self._view_transition_model.cost(view1, view2)

    def evaluate(self, view):
        return self._view_evaluator.evaluate(view)

    def goto(self, view):
        return self._view_controller.execute(view)

    def perform_action(self, view):
        return self._view_action.execute()
    
#######################################

class ViewGenerator(object):

    def generate(self):
        pass

#######################################

class ViewController(object):

    def execute(self, view):
        pass
    
#######################################
    
class ViewTransitionModel(object):

    def cost(self, view1, view2):
        pass

#######################################
    
class ViewEvaluator(object):

    def evaluate(self, view):
        pass

#######################################
    
class ViewAction(object):

    def execute(self):
        pass

