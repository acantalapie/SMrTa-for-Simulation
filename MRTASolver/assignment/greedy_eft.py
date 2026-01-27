import re
from .base import AssignmentPolicy
import math

class GreedyEarliestFinish(AssignmentPolicy):

    def __init__(self, room_graph, fidelity, action_time, **_):
        self.room_graph = room_graph
        self.fidelity = fidelity
        self.action_time = action_time
    
    def _dist(self, r1, r2):
        # Debe ser consistente con DistFunc del SMT
        return int(math.ceil(self.room_graph[r1][r2] / self.fidelity))

    # def select_agent(self, task, state, curr_time, deadline):
    #     best_a, best_finish = None, None
        
    #     for a in range(state.num_agents):
    #         # (R1) Capacidad: conservador (carga actual + tareas nuevas asignadas)
    #         if state.active_load[a] + state.future_load[a] >= state.capacity:
    #             continue

    #         # (R2) DPs disponibles: 2 (pickup + dropoff) deben estar libres     
    #         free_slots = state.num_aps - state.next_free_dp[a]
    #         if state.future_actions[a] + 2 > free_slots:
    #             continue

    #         available = max(state.last_time[a], curr_time)

    #         dist1 = self._dist(state.last_room[a], task.start)
    #         dist2 = self._dist(task.start, task.end)

    #         t_pick = available + dist1 + self.action_time
    #         t_drop = t_pick + dist2 + self.action_time

    #         # (R3) Deadline: la tarea debe poder completarse antes del deadline
    #         if t_drop > deadline:
    #             continue

    #         # Greedy: elige el que termina antes
    #         if best_finish is None or t_drop < best_finish:
    #             best_finish, best_a  = t_drop, a

    #     # if best_a is None:
    #     #     raise RuntimeError(
    #     #         f"[GEF] Tarea {task.id} imposible en curr_time={curr_time}: "
    #     #         f"start={task.start} end={task.end} deadline={deadline}. "
    #     #         f"Ningún agente cumple (capacidad/DP/deadline)."
    #     #     )
        
    #     return best_a  

    def select_agent(self, task, state, curr_time, deadline):
        reasons = {}  # agent_id -> list[str]
        best = None
        best_score = None

        for a in range(state.num_agents):
            reasons[a] = []

            # (R1) Capacidad
            if state.active_load[a] + state.future_load[a] >= state.capacity:
                reasons[a].append("capacity")
                continue

            # (R2) ETA conservador (tu lógica actual)
            prev_room = state.last_room[a]
            available = max(state.last_time[a], curr_time)
            t_pick = available + self._dist(prev_room, task.start) + self.action_time
            t_drop = t_pick + self._dist(task.start, task.end) + self.action_time

            # (R3) Deadline
            if deadline is not None and t_drop > deadline:
                reasons[a].append(f"deadline: eta={t_drop} > dl={deadline}")
                continue

            # score (EFT)
            score = t_drop
            if best_score is None or score < best_score:
                best_score = score
                best = a

        if best is None:
            # 🔥 print de diagnóstico
            print(f"\n[GREEDY NONE] task={task.start}->{task.end} dl={task.deadline} curr_time={curr_time}")
            for a in range(state.num_agents):
                print(f"  agent {a}: last_room={state.last_room[a]} last_time={state.last_time[a]} "
                    f"active={state.active_load[a]} future={state.future_load[a]} -> {reasons[a]}")
            return None

        return best

    def on_commit(self, agent_id, task, state, curr_time=None, deadline=None):
        """
        Actualización de estado tras asignar task a agent_id.
        En append-only asumimos que el pickup se abrirá y el drop se cerrará más adelante.
        Para aproximar la carga activa, incrementamos en 1 la carga del agente.
        """        
        # Estimación temporal coherente (sin usar la sala ya actualizada)
        prev_room = state.last_room[agent_id]
        available = max(state.last_time[agent_id], curr_time if curr_time is not None else 0)

        t_pick = available + self._dist(prev_room, task.start) + self.action_time
        t_drop = t_pick + self._dist(task.start, task.end) + self.action_time

        state.last_time[agent_id] = t_drop
        state.last_room[agent_id] = task.end