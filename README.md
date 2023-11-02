# \[AutoSVT\] Corner Case Discovery Algorithm 
This project presents a meta-heuristic algorithm aimed at discovering corner cases for autonomous driving systems within a simulator. It is designed based on several empirical insights, which provide guidance for both seed scenarios and mutations. This significantly reduces the search dimensions of scenarios and enhances the efficiency of corner case discovery, especially in adverse weather conditions.

Please visit [AutoSVT](https://idslab-autosec.github.io/) for more information.

## Installation


## Getting Started
Run CARLA, Apollo and bridge. For detailed steps, please refer to [AutoSVT-Bridge](https://github.com/idslab-autosec/AutoSVT-Carla-Apollo-Bridge).

### Discovering corner cases
You can use `discover.py` to find new corner cases, and for information on the relevant parameters, please check `discover.py -h`.

```bash
python discover.py -n 10 -m 8 --rear --small
```

### Corner cases reproduction
The corner cases we've already found are stored in `corner_case/`, and you can reproduce these corner cases using `replay.py`.

```bash
python replay.py [FILE]
```