import subprocess
import argparse
import sys
from pathlib import Path

def confirm_step(message, force):
    """Handles the y/n logic for each step."""
    if force:
        return True
    user_input = input(f"{message} [y/N]: ")
    if user_input.lower() != 'y':
        print("Step skipped or aborted by user.")
        return False
    return True

def get_mount_logic(device, force):
    dev_path = Path(f"/dev/{device}")
    if not dev_path.exists():
        print(f"Error: Device /dev/{device} not found.")
        sys.exit(1)

    # Check if already mounted
    result = subprocess.run(
        ["lsblk", "-no", "MOUNTPOINT", f"/dev/{device}"],
        capture_output=True, text=True
    ).stdout.strip()

    if result:
        return result

    # Mount confirmation
    if confirm_step(f"Device /dev/{device} is not mounted. Mount it to /mnt?", force):
        try:
            subprocess.run(["sudo", "mount", f"/dev/{device}", "/mnt"], check=True)
            return "/mnt"
        except subprocess.CalledProcessError as e:
            print(f"Error: Failed to mount. {e}")
            sys.exit(1)
    else:
        print("Cannot proceed without mounting.")
        sys.exit(0)

def get_sd_card_info():
    # We remove -d to see the 'tree' of partitions
    # We add FSTYPE to see which one is mountable
    cmd = "lsblk -e 7 -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT"
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, check=True)
        return result.stdout
    except subprocess.CalledProcessError as e:
        return f"Error: {e.stderr}"

def inject_configs(bot_id, device_name, force):
    bot_id = 'bot'+ bot_id
    # 1. Mount Phase
    mount_path = get_mount_logic(device_name, force)
    
    source_dir = Path(f"configs/{bot_id}")
    dest_dir = Path(mount_path)
    files_to_copy = ["user-data", "network-config"]

    # 2. Writing Phase
    if confirm_step(f"Overwrite files on {device_name} with configs from {bot_id}?", force):
        if not source_dir.exists():
            print(f"Error: Source {source_dir} not found.")
            sys.exit(1)
            
        try:
            for file_name in files_to_copy:
                src, dst = source_dir / file_name, dest_dir / file_name
                if src.exists():
                    subprocess.run(["sudo", "cp", str(src), str(dst)], check=True)
                    print(f"Overwrote: {dst}")
                else:
                    print(f"Warning: {file_name} missing from source.")
            
            print("Syncing filesystem...")
            subprocess.run(["sync"], check=True)
        except Exception as e:
            print(f"Write error: {e}")
            sys.exit(1)

    # 3. Unmount Phase
    if confirm_step(f"Unmount /dev/{device_name} now?", force):
        try:
            subprocess.run(["sudo", "umount", f"/dev/{device_name}"], check=True)
            print(f"Unmounted {device_name}. Safe to remove.")
        except subprocess.CalledProcessError as e:
            print(f"Unmount failed (device might be busy): {e}")

    print("\nProcess Complete")
    print("Reminder: Forget Wifi & Upload firmware (1.0.0 for ROS2 Jazzy FastDDS)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inject configs with granular confirmations")
    # Make bot_id and disk optional in the parser so we can prompt for them
    parser.add_argument("bot_id", nargs='?', help="Robot number (e.g., 03)")
    parser.add_argument("disk", nargs='?', help="Partition name (e.g., sdb1)")
    parser.add_argument("-y", "--yes", action="store_true", help="Bypass all confirmations")

    args = parser.parse_args()

    # If parameters are missing, show the drive info and ask the user
    if not args.bot_id or not args.disk:
        print("\n--- Current Disk Information ---")
        print(get_sd_card_info())
        print("--------------------------------\n")
        
        if not args.bot_id:
            args.bot_id = input("Enter Robot ID (e.g., 05): ").strip()
        if not args.disk:
            args.disk = input("Enter Partition Name (e.g., sdb1): ").strip()

    # Final check before proceeding
    if not args.bot_id or not args.disk:
        print("Error: Robot ID and Disk Partition are required.")
        sys.exit(1)

    inject_configs(args.bot_id, args.disk, args.yes)