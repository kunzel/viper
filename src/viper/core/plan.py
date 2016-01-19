import logging; logger = logging.getLogger("viper." + __name__)

from view import View

class Plan(object):

    def __init__(self, ID):
        self.ID = ID
        self.views = []
        self.reward = 0
        self.ind_reward = 0
        self.cost = 0
        self.planning_time = 0

    def append(self, view):
        self.views.append(view)

    def pop(self):
        self.views.pop()
        
