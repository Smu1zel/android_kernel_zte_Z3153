#!/usr/bin/env python3
# Script to automatically convert local headers included with angle brackets <>
# to double quotes "", resolving missing header errors in standalone builds.

import os
import re
import sys

# Only scan and fix custom MediaTek/ZTE vendor paths to keep standard Linux subsystems untouched
VENDOR_PATHS = [
    'drivers/misc/mediatek',
    'drivers/power/supply/mediatek',
    'drivers/watchdog/mediatek',
    'drivers/mmc/host/mediatek',
    'drivers/devfreq',
    'drivers/vendor',
    'sound/soc/mediatek'
]

def fix_local_includes(kernel_dir, dry_run=False):
    pattern = re.compile(r'#\s*include\s*<([^>/\\]+)>')
    modified_count = 0
    total_files_scanned = 0

    action_word = "Scanning (dry-run)" if dry_run else "Scanning and fixing"
    print(f"=== {action_word} local headers included with <...> in {kernel_dir} ===")

    for root, dirs, files in os.walk(kernel_dir):
        # Calculate relative path from kernel_dir using forward slashes
        rel_dir = os.path.relpath(root, kernel_dir).replace('\\', '/')
        
        # Check if the directory belongs to custom vendor/MediaTek paths
        is_vendor = any(rel_dir.startswith(p) for p in VENDOR_PATHS)
        if not is_vendor:
            continue

        for name in files:
            if name.endswith(('.c', '.h')):
                total_files_scanned += 1
                path = os.path.join(root, name)
                
                try:
                    with open(path, 'r', errors='ignore') as f:
                        lines = f.readlines()
                    
                    modified = False
                    new_lines = []
                    for line in lines:
                        match = pattern.search(line)
                        if match:
                            header_name = match.group(1)
                            local_header_path = os.path.join(root, header_name)
                            if os.path.exists(local_header_path):
                                line = line.replace(f"<{header_name}>", f'"{header_name}"')
                                modified = True
                        new_lines.append(line)
                        
                    if modified:
                        if not dry_run:
                            with open(path, 'w', errors='ignore') as f:
                                f.writelines(new_lines)
                            print(f"Fixed: {os.path.relpath(path, kernel_dir)}")
                        else:
                            print(f"Would fix: {os.path.relpath(path, kernel_dir)}")
                        modified_count += 1
                        
                except Exception as e:
                    print(f"Error processing {path}: {e}")

    print("\n=== Scan Complete ===")
    print(f"Total files scanned: {total_files_scanned}")
    if dry_run:
        print(f"Total files that would be modified: {modified_count}")
    else:
        print(f"Total files fixed: {modified_count}")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    kernel_path = os.path.join(script_dir, "kernel-4.9")
    
    dry_run = "--dry-run" in sys.argv
    
    if os.path.exists(kernel_path):
        fix_local_includes(kernel_path, dry_run=dry_run)
    else:
        print(f"Error: Could not find kernel-4.9 directory at {kernel_path}")
