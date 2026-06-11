import math


def encode_move(seq: int, angles_rad) -> str:
    """
    Encode joint angles thành lệnh $M cho STM32.

    angles_rad: iterable [Q1, Q2, Q3, Q4] theo radian
      Q1 (yaw)      → angle1 (Motor 1)
      Q2 (shoulder) → angle2 (Motor 2)
      Q3 (elbow)    → angle3 (Motor 3)
      Q4 (wrist)    → reserved, chưa gửi (firmware chưa hỗ trợ M4)

    Trả về chuỗi "$M,{seq},{a1},{a2},{a3}\n" sẵn sàng gửi qua serial.
    Góc được làm tròn sang centidegree (×100), clamp [-18000, 18000].
    seq tự động wrap 0–255.
    """
    q1, q2, q3 = float(angles_rad[0]), float(angles_rad[1]), float(angles_rad[2])

    def _to_centideg(rad: float) -> int:
        return max(-18000, min(18000, round(math.degrees(rad) * 100)))

    a1 = _to_centideg(q1)
    a2 = _to_centideg(q2)
    a3 = _to_centideg(q3)
    s = int(seq) & 0xFF
    return f"$M,{s},{a1},{a2},{a3}\n"


def decode_response(line: str) -> dict | None:
    """
    Parse một dòng phản hồi từ STM32.

    Hỗ trợ:
      $A,<seq>                       → {"type": "ACK",  "seq": int}
      $D,<seq>,<a1>,<a2>,<a3>        → {"type": "DONE", "seq": int,
                                         "angles_centideg": [a1, a2, a3]}
      $E,<seq>,<CODE>                → {"type": "ERR",  "seq": int, "code": str}

    Trả về None nếu dòng không hợp lệ hoặc không nhận ra.
    """
    line = line.strip()
    if not line.startswith("$") or len(line) < 3:
        return None

    parts = line.split(",")
    tag = parts[0]  # "$A", "$D", "$E"

    try:
        if tag == "$A" and len(parts) == 2:
            return {"type": "ACK", "seq": int(parts[1])}

        if tag == "$D" and len(parts) == 5:
            return {
                "type": "DONE",
                "seq": int(parts[1]),
                "angles_centideg": [int(parts[2]), int(parts[3]), int(parts[4])],
            }

        if tag == "$E" and len(parts) == 3:
            return {"type": "ERR", "seq": int(parts[1]), "code": parts[2]}

    except (ValueError, IndexError):
        pass

    return None
