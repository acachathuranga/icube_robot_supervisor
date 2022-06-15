class Task:
    """
        Task Interface

        Virtual Methods:
            getState()                  :       String State
            execute(String Command)     :       Bool Success, String Msg          
    """

    class TaskState:
        Ok = 1
        Stopped = 2
        Error = 3

    def getState(self):
        """ Returns TaskState and Current state data
        """
        raise NotImplementedError()

    def __call__(self, *args):
        """ Returns Task State and state data
        """
        raise NotImplementedError()