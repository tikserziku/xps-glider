# -*- coding: utf-8 -*-
"""author.py - optional author line for the printed sheets.

The name is NOT part of the project. It is read from <project>/author.local.json (git-ignored):
    {"author": "NAME SURNAME", "group": "grupė XXXX"}
No file, or the environment variable GLIDER_ANON=1  ->  the sheets are drawn without an author line.
"""
import json
import os

_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "author.local.json")


def author():
    """(author, group) or ("", "")."""
    if os.environ.get("GLIDER_ANON") == "1" or not os.path.exists(_FILE):
        return "", ""
    d = json.load(open(_FILE, encoding="utf-8"))
    return d.get("author", "").strip(), d.get("group", "").strip()


def author_line(sep=" · "):
    a, g = author()
    return sep.join(x for x in (a, g) if x)
