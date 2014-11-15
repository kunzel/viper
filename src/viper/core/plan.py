import logging; logger = logging.getLogger("viper." + __name__)

from view import View

class Plan(object):

    def __init__(self, ID):
        self.ID = ID
        self.views = [] 

    def append(self, view):
        self.views.append(view)
        
