import collections
import time
import socket

# ===== CONFIG =====
ESP32_LISTEN_IP = "0.0.0.0"
ESP32_PORT = 8000

STUDIO_IP = "127.0.0.1"
STUDIO_PORT = 9000

MEDIAN_N = 9
EMA_ALPHA = 0.2
UPDATE_PERIOD = 1.0
# ==================

# ---------- ESP32 TCP SERVER ----------
esp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
esp_sock.bind((ESP32_LISTEN_IP, ESP32_PORT))
esp_sock.listen(1)

print(f"Waiting for ESP32 on {ESP32_LISTEN_IP}:{ESP32_PORT} ...")
esp_conn, esp_addr = esp_sock.accept()
print(f"ESP32 connected from {esp_addr}")

# ---------- SERIAL STUDIO TCP SERVER ----------
studio_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
studio_sock.bind((STUDIO_IP, STUDIO_PORT))
studio_sock.listen(1)

print(f"Waiting for Serial Studio on {STUDIO_IP}:{STUDIO_PORT} ...")
studio_conn, studio_addr = studio_sock.accept()
print(f"Serial Studio connected from {studio_addr}\n")

# ---------- FILTER STATE ----------
buf = collections.deque(maxlen=MEDIAN_N)
ema_val = None
last_publish = 0

def median(lst):
    s = sorted(lst)
    return s[len(s)//2]

print("Wireless TCP bridge running...\n")

rx_buffer = ""   # <-- THIS IS THE FIX

while True:
    try:
        chunk = esp_conn.recv(1024).decode(errors="ignore")
        if not chunk:
            continue

        rx_buffer += chunk

        # Process full lines only
        while "\n" in rx_buffer:
            line, rx_buffer = rx_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 6:
                continue

            volume = float(parts[0])
            percent = float(parts[1])
            sec_since = int(parts[2])
            sec_left = int(parts[3])
            orientation_ok = (parts[4] == "1")
            sip_detected = (parts[5] == "1")

            # ---- UPDATE FILTER ONLY IF ORIENTATION OK ----
            if orientation_ok:
                buf.append(volume)
                med = median(buf)

                if ema_val is None:
                    ema_val = med
                else:
                    ema_val = EMA_ALPHA * med + (1 - EMA_ALPHA) * ema_val

            if ema_val is None:
                continue

            now = time.time()
            if now - last_publish < UPDATE_PERIOD:
                continue
            last_publish = now

            stable_vol = round(ema_val)

            # ---- CSV OUTPUT (UNCHANGED) ----
            out_line = (
                f"{stable_vol},"
                f"{percent:.1f},"
                f"{sec_since},"
                f"{sec_left},"
                f"{1 if orientation_ok else 0},"
                f"{1 if sip_detected else 0}\n"
            )

            studio_conn.sendall(out_line.encode())

            # ---- DEBUG ----
            print(
                f"SEND → {stable_vol:4d} mL | "
                f"{percent:5.1f}% | "
                f"{sec_since:4d}s | "
                f"{sec_left:4d}s | "
                f"ORI={'1' if orientation_ok else '0'} | "
                f"SIP={'1' if sip_detected else '0'}"
            )

    except KeyboardInterrupt:
        print("\nStopped by user.")
        break

    except Exception as e:
        print("ERROR:", e)
        time.sleep(0.1)
