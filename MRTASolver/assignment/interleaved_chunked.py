from .base import AssignmentPolicy
import math 

class GreedyChunckedPickDrop(AssignmentPolicy):

    def __init__(self, room_graph, fidelity, action_time, capacity, **_):
        self.room_graph = room_graph
        self.fidelity = fidelity
        self.action_time = action_time
        self.capacity = capacity

    def _dist(self, r1, r2):
        # Debe ser consistente con DistFunc del SMT
        return int(math.ceil(self.room_graph[r1][r2] / self.fidelity))

    def build_plan(self, tasks, state, curr_time, base_offset):
        num_agents = state.num_agents
        num_tasks = len(tasks)

        plan_actions = {a: [] for a in range(num_agents)}
        task_to_agent = [-1] * num_tasks

        # (1) Cerrar pickups abiertos -> generar dropoffs de tareas antiguas
        for a in range(num_agents):
            for pickup_id in state.open_pickups[a]:
                plan_actions[a].append(pickup_id + 1)
        # (2) Asignación simple de tareas nuevas (por deadline ascendente)
        order = sorted(range(num_tasks), key = lambda i: tasks[i].get_deadline(10*9)) #---------------------------> Revisar get deadline

        # Limite de taras por DPs disponibles
        max_taks_by_agent = [max(0, (state.num_aps - state.next_free_dp[a] - len(plan_actions[a]))//2) for a in range(num_agents)]

        assigned = [[] for _ in range(num_agents)]

        for i in order:
            deadline = tasks[i].get_deadline(10*9)
            best_a , best_score = None, None

            for a in range(num_agents):
                if len(assigned[a]) >= max_taks_by_agent[a]:
                    continue
                
                # Scoring EFT aproximado asumiendo "directo" (solo para signar agente)
                prev_room = state.last_room[a]
                available = max(state.last_time[a], curr_time)
                t_pick = available + self._dist(prev_room, tasks[i].start) + self.action_time
                t_drop = t_pick + self._dist(tasks[i].start, tasks[i].end) + self.action_time

                if t_drop > deadline:
                    continue
                if best_score is None or t_drop < best_score:
                    best_score, best_a = t_drop, a
            if best_a is None:
                return None # Deja que el solver caiga a UNSAT

            assigned[best_a].append(i)
            task_to_agent[i] = best_a

        # (3) Construir plan final con pickups y dropoffs intercalados
        for a in range(num_agents):
            # Capacidad libre tras ejecutar los dropoffs pendientes
            free_cap = max(1, self.capacity - state.active_load[a])

            # Trocear en grupos de tamaño free_cap
            idxs = assigned[a]
            for s in range (0, len(idxs), free_cap):
                chunk = idxs[s:s+free_cap]
                # Pickups del chunk
                for ti in chunk:
                    pickup_id = base_offset + 2 * ti
                    plan_actions[a].append(pickup_id)
                for ti in chunk:
                    dropoff_id = base_offset + 2 * ti + 1
                    plan_actions[a].append(dropoff_id)

        return plan_actions, task_to_agent 

