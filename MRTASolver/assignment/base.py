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

    def build_plan(self, task, state, curr_time, base_offset):
        """
        Si la policy devuelve (plan_actions, task_to_agent), el solver usará ese plan tal cual.
            - plan_actions: dict[int -> list[int]]  (agent_id -> action_id sequence)
            - task_to_agent: list[int]  (solo para tareas del batch actual)
        Si devuelve None, se usa el framework iterativo (select_agent + _commit_assignment).
        """
        return None