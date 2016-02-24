#!/bin/sh
grep -n -e FIXME: -e TODO: v_repExtOctomap.cpp | sed -e 's/:[[:space:]]*/:	/'
