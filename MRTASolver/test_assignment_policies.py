# test_all_assignment_policies.py

"""
Test global para comparar múltiples métodos de asignación de tareas
bajo el mismo entorno MRTA + SMT (fixed-plan).

Permite:
- probar varias AssignmentPolicy
- comparar SAT/UNSAT
- comparar tiempos de resolución
- reutilizar exactamente el mismo solver y escenario
"""

import math
from .objects import Robot, Task
from .MRTASolver import MRTASolver
from .SolverInterface import Result

# Policies
from .assignment.greedy_eft import GreedyEarliestFinish
from .assignment.round_robin import RoundRobinAssigment


def simple_room_graph():
    # 4 habitaciones, matriz simétrica con diagonales 0
    return [
        [0, 2, 5, 6],
        [2, 0, 3, 5],
        [5, 3, 0, 2],
        [6, 5, 2, 0],
    ]

def build_solver(room_graph, agents, tasks_stream):
    room_graph = simple_room_graph()
    agents = [Robot(0, 0), Robot(1, 1)]

    tasks_stream = [
        ([Task(0, 2, 3, deadline=30), Task(1, 3, 0, deadline=35)], 0),
        ([Task(2, 2, 1, deadline=50), Task(3, 0, 2, deadline=55)], 10),
    ]


    num_aps = 10
    aps_list = [6, 8, 10]
    capacity = 2

    solver = MRTASolver(
        solver_name="bitwuzla",
        theory="QF_UFBV",
        agents=agents,
        tasks_stream=tasks_stream,
        room_graph=room_graph,
        capacity=capacity,
        num_aps=num_aps,
        fidelity=1,
        free_action_points=True,
        timeout=None,
        default_deadline=1000,
        aps_list=aps_list,
        incremental=True,
        debug=False,
    )
    # 🔧 FIX: inicializar llegadas reales (tester no simula ejecución real)
    solver.actual_agent_arrivals = [[] for _ in range(len(agents))]

    return solver

def format_solution_table(sol, tasks_stream):  
    """  
    Formatea la solución del solver en una tabla legible.  
    
    Args:  
        sol: Diccionario de solución devuelto por extract_model()  
        tasks_stream: Stream de tareas usado para generar la solución  
    """  
    # Calcular duraciones para cada tarea  
    task_durations = []  
    task_info = []  
    
    for batch_idx, (tasks, _) in enumerate(tasks_stream[:len(sol['ts'])]):
        for task_idx, task in enumerate(tasks):  
            pickup_time = sol['ts'][batch_idx][task_idx]  
            dropoff_time = sol['td'][batch_idx][task_idx]  
            agent_id = sol['t2a'][batch_idx][task_idx]  
            duration = dropoff_time - pickup_time  
            
            task_info.append({  
                'task_id': f"T{batch_idx}-{task_idx}",  
                'origin': task.start,  
                'destination': task.end,  
                'agent': f"Agente {agent_id + 1}",  
                'pickup': pickup_time,  
                'dropoff': dropoff_time,  
                'duration': duration  
            })  
    
    # Crear tabla formateada  
    print("┌───────────────────────┬─────────────┬─────────────┬──────────────┬──────────┐")  
    print("│ Tarea                 │ Agente      │ Pickup (T)  │ Dropoff (T)  │ Duración │")  
    print("├───────────────────────┼─────────────┼─────────────┼──────────────┼──────────┤")  
    
    for task in task_info:  
        task_str = f"{task['task_id']} [{task['origin']}, {task['destination']}]"  
        print(f"│ {task_str:<21} │ {task['agent']:<11} │ {task['pickup']:<11} │ {task['dropoff']:<12} │ {task['duration']:<8} │")  
    
    print("└───────────────────────┴─────────────┴─────────────┴──────────────┴──────────┘")  
    
    # return task_info   

def run_policy(policy_name, solver, tasks_stream, agents):
    print(f"\n==============================")
    print(f"Policy: {policy_name}")
    print(f"==============================")

    previous_sol = None
    partial_stream = []
    num_tasks_total = 0
    curr_max_deadline = 0

    if policy_name == "GreedyEFT":
        policy = GreedyEarliestFinish(solver.room_graph, solver.fidelity, solver.action_time) 
    else:
        policy = RoundRobinAssigment()

    for k, (tasks, curr_time) in enumerate(tasks_stream):
        partial_stream.append((tasks, curr_time))
        num_tasks_total += len(tasks)
        curr_max_deadline = max(curr_max_deadline, max(t.get_deadline(1000) for t in tasks))
        curr_max_time = curr_max_deadline + solver.max_travel_time

        # (1) Greedy decide el plan
        plan_actions, task_to_agent, _ = solver.assign_tasks(
            tasks=tasks,
            curr_time=curr_time,
            previous_sol=previous_sol,
            assignment_policy=policy,
        )

        # (2) SMT valida el plan fijado
        solver.add_task_constraints_fixed_plan(
            agents=agents,
            tasks=tasks,
            num_aps=solver.num_aps,
            curr_time=curr_time,
            curr_max_time=curr_max_time,
            task_set_k=k,
            sol=previous_sol,
            fidelity=1,
            plan_actions=plan_actions,
            task_to_agent=task_to_agent,
        )

        # (3) Solve con tus assumptions / aps_list
        min_dps = solver.get_min_dps(len(agents), num_tasks_total)
        times, results, result, used_solver = solver.solve_task_allocation(num_tasks_total, min_dps)

        print(f"Batch {k} @ t={curr_time} -> {result}")

        if result != Result.sat:
            print("UNSAT → política rechazada")
            return

        # (4) Extraer solución (y consistencia con la anterior)
        current_sol = solver.validate_task_allocation(
            prev_sol=previous_sol,
            partial_task_stream=partial_stream,
            solver=used_solver,
            curr_time=curr_time,
            curr_max_time=curr_max_time,
        )
        previous_sol = current_sol

    print("✔ Política válida en todos los batches\n")
    format_solution_table(current_sol, tasks_stream)
    
def main():
    room_graph = simple_room_graph()

    agents = [Robot(0, 0), Robot(1, 1)]

    tasks_stream = [
        ([Task(0, 2, 3, deadline=30), Task(1, 3, 0, deadline=35)], 0),
        ([Task(2, 2, 1, deadline=50), Task(3, 0, 2, deadline=55)], 10),
    ]

    policies = ["GreedyEFT", "RoundRobin"]

    for policy_name in policies:
        base_solver = build_solver(room_graph, agents, tasks_stream)  # ✅ NUEVO solver
        run_policy(policy_name, base_solver, tasks_stream, agents)

if __name__ == "__main__":
    main()
