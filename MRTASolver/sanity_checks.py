# sanity_checks.py


class SanityCheckError(Exception):
    pass


def is_pickup(action_id, num_agents):
    return action_id >= num_agents and (action_id - num_agents) % 2 == 0


def is_dropoff(action_id, num_agents):
    return action_id >= num_agents and (action_id - num_agents) % 2 == 1


def task_index_from_action(action_id, num_agents):
    """
    Devuelve el índice de tarea local (0..num_tasks-1) a partir de un action_id.
    """
    return (action_id - num_agents) // 2


# --------------------------------------------------
# CHECKS
# --------------------------------------------------

def check_action_id_ranges(plan_actions, base_offset, num_tasks):
    """
    Verifica que:
      - todos los action_id sean >= num_agents
      - pertenezcan al rango de tareas del batch
    """
    min_task_id = base_offset
    max_task_id = base_offset + 2 * num_tasks - 1

    for agent_id, actions in plan_actions.items():
        for act in actions:
            if act < min_task_id:
                raise SanityCheckError(
                    f"Action ID inválido (< num_agents): "
                    f"agent={agent_id}, action={act}"
                )
            if act > max_task_id:
                raise SanityCheckError(
                    f"Action ID fuera del batch: "
                    f"agent={agent_id}, action={act}, "
                    f"esperado <= {max_task_id}"
                )
    return True


def check_pickup_before_dropoff(plan_actions, num_agents):
    """
    Para cada agente y cada tarea:
      - pickup aparece antes que dropoff
      - no hay dropoff sin pickup
    """
    for agent_id, actions in plan_actions.items():
        pos = {act: i for i, act in enumerate(actions)}

        for act in actions:
            if not is_dropoff(act, num_agents):
                continue

            pickup_id = act - 1

            if pickup_id not in pos:
                task_idx = task_index_from_action(act, num_agents)
                raise SanityCheckError(
                    f"Dropoff sin pickup: agent={agent_id}, "
                    f"task={task_idx}, dropoff={act}"
                )

            if pos[pickup_id] > pos[act]:
                task_idx = task_index_from_action(act, num_agents)
                raise SanityCheckError(
                    f"Dropoff antes de pickup: agent={agent_id}, "
                    f"task={task_idx}, pickup={pickup_id}, dropoff={act}"
                )
    return True


def check_task_assigned_once(task_to_agent, num_agents, num_tasks):
    """
    Verifica que:
      - task_to_agent sea una lista
      - tenga longitud num_tasks
      - cada tarea esté asignada a un agente válido
    """
    if not isinstance(task_to_agent, list):
        raise SanityCheckError(
            "task_to_agent debe ser una lista indexada por task_index"
        )

    if len(task_to_agent) != num_tasks:
        raise SanityCheckError(
            f"task_to_agent longitud incorrecta: "
            f"{len(task_to_agent)} != {num_tasks}"
        )

    for task_idx, agent_id in enumerate(task_to_agent):
        if not isinstance(agent_id, int):
            raise SanityCheckError(
                f"Agente no entero en task {task_idx}: {agent_id}"
            )
        if agent_id < 0 or agent_id >= num_agents:
            raise SanityCheckError(
                f"Agente inválido en task {task_idx}: {agent_id}"
            )
    return True


def check_capacity_conservative(state):
    """
    Verifica que el greedy respete:
      active_load + future_load <= capacity
    """
    for agent_id in range(len(state.active_load)):
        if state.active_load[agent_id] + state.future_load[agent_id] > state.capacity:
            raise SanityCheckError(
                f"Capacidad violada: agent={agent_id}, "
                f"active={state.active_load[agent_id]}, "
                f"future={state.future_load[agent_id]}, "
                f"capacity={state.capacity}"
            )
    return True


# --------------------------------------------------
# ENTRY POINT
# --------------------------------------------------

def run_all_sanity_checks(
    plan_actions,
    task_to_agent,
    state,
    num_agents,
    num_tasks,
    base_offset,
    verbose=True,
):
    """
    Ejecuta todos los sanity checks del plan greedy.
    """
    if verbose:
        print("🧪 Running sanity checks...")

    check_action_id_ranges(plan_actions, base_offset, num_tasks)
    check_pickup_before_dropoff(plan_actions, num_agents)
    check_task_assigned_once(task_to_agent, num_agents, num_tasks)
    check_capacity_conservative(state)

    if verbose:
        print("✅ Sanity checks OK")

    return True
