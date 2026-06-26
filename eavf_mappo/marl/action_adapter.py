from __future__ import annotations

from eavf_mappo.envs.multi_robot_exploration_env import CONTINUE_ACTION, RELEASE_ACTION

__all__ = ["CONTINUE_ACTION", "RELEASE_ACTION", "decode_action"]


def decode_action(action_idx: int, topk_tokens) -> int:
    """Map an actor index in `K+2` space to the integer action expected by env.step.

    Slots `0..K-1` select a valid frontier token. Slot `K` continues the
    current target, and slot `K+1` releases/waits. Invalid token slots fall back
    to continue so masked-action mistakes remain harmless at the env boundary.
    """
    idx = int(action_idx)
    k = len(topk_tokens)
    if 0 <= idx < k:
        token = topk_tokens[idx]
        if getattr(token, "valid", False):
            return int(token.frontier_id)
        return CONTINUE_ACTION
    if idx == k:
        return CONTINUE_ACTION
    if idx == k + 1:
        return RELEASE_ACTION
    return CONTINUE_ACTION
