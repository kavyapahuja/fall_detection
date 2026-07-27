import requests
import os
from dotenv import load_dotenv

load_dotenv()

TELEGRAM_BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
TELEGRAM_CHAT_IDS = os.getenv("TELEGRAM_CHAT_IDS", "").split(",")

def send_telegram_alert(message):
    url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage"
    success = True
    for chat_id in TELEGRAM_CHAT_IDS:
        chat_id = chat_id.strip()
        if not chat_id:
            continue
        payload = {
            "chat_id": chat_id,
            "text": message
        }
        try:
            response = requests.post(url, data=payload, timeout=5)
            if not response.ok:
                success = False
        except Exception as e:
            print(f"Failed to send telegram alert to {chat_id}: {e}")
            success = False
    return success


if __name__ == "__main__":
    success = send_telegram_alert("test alert from fall detection system")
    print("sent successfully!" if success else "failed to send")