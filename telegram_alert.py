import requests
import os 
from dotenv import load_dotenv

load_dotenv()

TELEGRAM_BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
TELEGRAM_CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")


def send_telegram_alert(message):
    url = f"https://api.telegram.org/bot8897515091:AAEJjQzxp7cqDUYSH36aYAgdhhGBCfS8EmM/sendMessage"
    payload = {
        "chat_id" : TELEGRAM_CHAT_ID,
        "text" : message 
    }

    try:
        response = requests.post(url, data=payload, timeout=5)
        return response.ok
    except Exception as e:
        print(f"Failed to send telegram alert: {e}")
        return False

if __name__ == "__main__":
   success = send_telegram_alert("test alert from fall detection system")
   print("sent successfully!" if success else "failed to send")
