from abc import ABC, abstractmethod

class AssignmentPolicy(ABC):
    """
    Interfaz base para cualquier método de asignación
    """

    def task_order(self, tasks, state):
        """
        Devuelve el orden en el que se procesan las tareas.
        Por defecto: orden de llegada
        """

        return range(len(tasks))

    @abstractmethod
    def select_agent(self, task, state, curr_time):
        """
        Devuelve el agent_id al que asginar la tarea
        """
        pass
    
    def on_commit(self, task, state, curr_time):
        """
        Hook opcional tras asignar una tarea.
        """
        pass