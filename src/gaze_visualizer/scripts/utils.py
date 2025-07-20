import os
from datetime import datetime

BASE_RESULTS_DIR = "/home/ros/eyegaze_ws/results"
CURRENT_ID_FILE = os.path.join(BASE_RESULTS_DIR, "current_participant.txt")


def get_or_create_participant_id():
    """Prompt for participant ID if not already stored."""
    if os.path.exists(CURRENT_ID_FILE):
        with open(CURRENT_ID_FILE, 'r') as f:
            participant_id = f.read().strip()
            print(f"Using existing Participant ID: {participant_id}")
    else:
        participant_id = input("Enter a unique participant ID: ").strip()
        with open(CURRENT_ID_FILE, 'w') as f:
            f.write(participant_id)
        print(f"Participant ID '{participant_id}' recorded.")

    return participant_id


def create_session_folder(game_type):
    """Creates a time-stamped results directory for the specified game under the participant's folder."""
    participant_id = get_or_create_participant_id()
    game_folder = os.path.join(BASE_RESULTS_DIR, participant_id, f"{game_type}_game_result")
    os.makedirs(game_folder, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(game_folder, f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)

    return session_dir


def clear_participant_id():
    """Removes the current participant ID file after the full session is done."""
    if os.path.exists(CURRENT_ID_FILE):
        os.remove(CURRENT_ID_FILE)
        print("Cleared current participant ID.")
    else:
        print("No participant ID to clear.")
