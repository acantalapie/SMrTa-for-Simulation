from .base import AssignmentPolicy
import math

class GreedyEarliestFinish(AssignmentPolicy):

    def __init__(self, room_graph, fidelity, action_time, **_):
        self.room_graph = room_graph
        self.fidelity = fidelity
        self.action_time = action_time
    
    def select_agent(self, task, state, curr_time, deadline):
        best_a, best_finish = None, None
        
        for a in range(state.num_agents):
            available = max(state.last_time[a], curr_time)

            dist1 = math.ceil(self.room_graph[state.last_room[a]][task.start] / self.fidelity)
            dist2 = math.ceil(self.room_graph[task.start][task.end] / self.fidelity)

            t_pick = available + dist1 + self.action_time
            t_drop = t_pick + dist2 + self.action_time

            # Filtro duro por deadline
            if t_drop > deadline:
                continue

            if best_finish is None or t_drop < best_finish:
                best_finish, best_a  = t_drop, a

        if best_a is None:
            raise ValueError(
                f"Tarea {task.id} imposible: ningún agente cumple deadline={deadline}"
            )
        
        return best_a

    def on_commit(self, agent_id, task, state):
        state.last_room[agent_id] = task.end
