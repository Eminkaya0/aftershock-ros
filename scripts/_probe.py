#!/usr/bin/env python3
import sys, os, locale
print("PY:", sys.executable)
print("VER:", sys.version.split()[0])
print("ENC:", sys.stdout.encoding, "| PREF:", locale.getpreferredencoding(False))
for k in ("PYTHONPATH","PYTHONHOME","PYTHONIOENCODING","LANG","LC_ALL"):
    print(k, "=", os.environ.get(k))
