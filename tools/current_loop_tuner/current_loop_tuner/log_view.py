DEFAULT_LOG_MAX_LINES = 5000


def append_log_line(lines: list[str], line: str, max_lines: int = DEFAULT_LOG_MAX_LINES) -> list[str]:
    if max_lines < 1:
        raise ValueError("max_lines must be positive")
    return [*lines, line][-max_lines:]
