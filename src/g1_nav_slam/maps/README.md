# Physical maps are intentionally not published

This directory is the default location for four-file `slam_toolbox` map sets:

```text
<name>.pgm
<name>.yaml
<name>.posegraph
<name>.data
```

Real laboratory maps may reveal private building geometry, so `.gitignore` excludes them. Create your own map with `mapping.launch.py`, save it with `save_map.sh`, and pass its basename to `localization.launch.py`.
