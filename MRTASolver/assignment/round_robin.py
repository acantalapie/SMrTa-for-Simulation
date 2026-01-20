from .base import AssignmentPolicy

class RoundRobinAssigment(AssignmentPolicy):
    """
    Política de asignación Round-Robin.

    Asigna las tareas de forma cíclica entre los agentes disponibles,
    sin tener en cuenta:
      - tiempos de ejecución
      - distancias
      - carga previa
      - deadlines

    Esta política se utiliza principalmente como:
      - baseline estructural
      - referencia para comparar otros métodos
      - herramienta de depuración del pipeline (greedy + SMT)
    """
    def __init__(self, **_):
        self.counter = 0

    def select_agent(self, task, state, curr_time):
        agent = self.counter % state.num_agents
        self.counter += 1

        return agent
