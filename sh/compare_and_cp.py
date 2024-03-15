#!/usr/bin/env python3

"""
compare_ad_cp.py

synchronize content between two files.
"""

import filecmp
import shutil
import sys
import os

def compare_and_copy(f1: str, f2: str) -> None:
    if not filecmp.cmp(f1, f2):
        shutil.copyfile(f1, f2)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_and_cp.py <src> <dst>")
        sys.exit(1)
    
    f1, f2 = sys.argv[1], sys.argv[2]
    
    if os.path.exists(f1) and os.path.exists(f2):
        compare_and_copy(f1, f2)
        print(f"[compare_and_cp.py]: File {f1} synchronized with {f2}.")
    else:
        print("[ERROR][compare_and_cp.py]: One or both files do not exist.")
        sys.exit(1)
