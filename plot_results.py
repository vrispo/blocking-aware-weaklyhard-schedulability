import os
import json
import sys
import numpy as np
import matplotlib.pyplot as plt
import re
from matplotlib.ticker import MultipleLocator, FormatStrFormatter, MaxNLocator
import matplotlib as mpl
import csv

mpl.rcParams.update({
    "font.family": "serif",
    "font.size": 14,
    "axes.labelsize": 16,
    "axes.titlesize": 16,
    "legend.fontsize": 13,
    "xtick.labelsize": 12,

    "ytick.labelsize": 13,
    "lines.linewidth": 2.2,
    "lines.markersize": 7,
    "grid.alpha": 0.3,
    "figure.dpi": 300,
    "savefig.dpi": 300,
})

if len(sys.argv) < 2:
    print("Usage: python plot_results.py <experiment_folder>")
    sys.exit(1)

exp_dir = sys.argv[1]
plots_dir = os.path.join(exp_dir, "plots")
os.makedirs(plots_dir, exist_ok=True)

results_root = os.path.join(exp_dir, "results")

# -------------------------
# detect sweep parameter
# -------------------------
with open(os.path.join(exp_dir, "config.json"), "r") as f:
    cfg = json.load(f)

sweep = cfg["experiment"]["sweep_parameter"]

# -------------------------
# build plot title
# -------------------------
gen = cfg["generation"]
exp = cfg["experiment"]

# map config keys to readable labels
param_map = {
    "N": "$N$",
    "U": "$U$",
    #"T_min": "$T_{min}$",
    #"T_max": "$T_{max}$",
    #"N_mk": "$N_{mk}$",
    "m": "$m$",
    "k": "$k$",
    "crit_sec_perc": r"$\beta$%"
}

title_parts = []

for key, label in param_map.items():
    if key in gen and key != sweep:   # <-- exclude swept parameter
        title_parts.append(f"{label}={gen[key]}")

# experiment-level info (always included)
#title_parts.append(f"sets={exp['num_tasksets']}")

plot_title = " , ".join(title_parts)

# -------------------------
# human-readable axis labels
# -------------------------
label_map = {
    "U": r"Utilization ($U$)",
    "N": r"Number of tasks ($N$)",
    #"N_mk": r"Number of weakly-hard tasks ($N_{mk}$)",
    "m": r"Weakly-hard constraint ($m$)",
    "k": r"Weakly-hard constraint ($k$)",
    "crit_sec_perc": r"$\beta$ (%)"
}

xlabel = label_map.get(sweep, sweep)

# -------------------------
# regex for parsing tags
# -------------------------
def extract_value(tag, sweep):
    if sweep == "U":
        m = re.search(r"U_([0-9]+\.[0-9]+)", tag)
        return float(m.group(1)) if m else None
    elif sweep == "N":
        m = re.search(r"N_([0-9]+)", tag)
        return int(m.group(1)) if m else None
    elif sweep == "m":
        m = re.search(r"m_([0-9]+)", tag)
        return int(m.group(1)) if m else None
    elif sweep == "k":
        m = re.search(r"k_([0-9]+)", tag)
        return int(m.group(1)) if m else None
    elif sweep == "crit_sec_perc":
        m = re.search(r"crit_([0-9]+)", tag)
        return int(m.group(1)) if m else None
    elif sweep == "N_mk":
        m = re.search(r"N_mk_([0-9]+)", tag)
        return int(m.group(1)) if m else None
    return None

# -------------------------
# collect files
# -------------------------
files = []
for f in os.listdir(results_root):
    if f.startswith("results_") and f.endswith(".json"):
        files.append(f)

data = []

for f in files:
    tag = f.replace("results_", "").replace(".json", "")
    v = extract_value(tag, sweep)

    if v is None:
        continue

    with open(os.path.join(results_root, f), "r") as fp:
        j = json.load(fp)

    data.append((v, j))

# -------------------------
# sort
# -------------------------
data.sort(key=lambda x: x[0])

x = [d[0] for d in data]

our_avg = [d[1]["our_avg_t"] for d in data]
their_avg = [d[1]["their_avg_t"] for d in data]

our_min = [d[1]["our_min_t"] for d in data]
our_max = [d[1]["our_max_t"] for d in data]

their_min = [d[1]["their_min_t"] for d in data]
their_max = [d[1]["their_max_t"] for d in data]

our_sched = []
their_sched = []

for _, j in data:
    sched = j["schedulability"]
    our_sched.append(sum(s["sched_our"] for s in sched) / len(sched))
    their_sched.append(sum(s["sched_their"] for s in sched) / len(sched))

# -------------------------
# plot time
# -------------------------
plt.figure(figsize=(5,3.5))
plt.title(plot_title, fontsize=14)

scale = 1e6

our_avg   = np.array(our_avg) / scale
their_avg = np.array(their_avg) / scale

our_min = np.array(our_min) / scale
our_max = np.array(our_max) / scale

their_min = np.array(their_min) / scale
their_max = np.array(their_max) / scale

plt.plot(x, our_avg, marker='o', label="PA avg")
plt.fill_between(x, our_min, our_max, alpha=0.2)

plt.plot(x, their_avg, marker='x', label="RM avg")
plt.fill_between(x, their_min, their_max, alpha=0.12)

plt.xlabel(xlabel)
plt.ylabel("Time (s)")
plt.yscale("log")

#plt.legend()
#plt.legend(loc="best")
ax = plt.gca()
ax.legend(loc="best", borderaxespad=1.2)

ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(nbins=8, integer=True))
#ax.tick_params(axis='x', rotation=45)
#plt.xticks(ha='right')

plt.grid()
plt.tight_layout()

plt.savefig(os.path.join(plots_dir, "time.png"), dpi=300)
plt.savefig(os.path.join(plots_dir, "time.pdf"), bbox_inches="tight")
plt.close()

# -------------------------
# plot schedulability
# -------------------------
plt.figure(figsize=(5, 3.5))
plt.title(plot_title, fontsize=14)

plt.plot(x, our_sched, marker='o', label="PA")
plt.plot(x, their_sched, marker='x', label="RM")

plt.xlabel(xlabel)
plt.ylabel("Schedulability")
plt.ylim(-0.05, 1.05)

#plt.legend(loc="upper center", bbox_to_anchor=(0.88, 0.95))
ax = plt.gca()
ax.legend(loc="best", borderaxespad=1.2)

ax.xaxis.set_major_locator(MaxNLocator(nbins=8, integer=True))
#ax.tick_params(axis='x', rotation=45)
#plt.xticks(ha='right')

#plt.legend(loc="best")
plt.grid()
plt.tight_layout()

plt.savefig(os.path.join(plots_dir, "sched.png"), dpi=300)
plt.savefig(os.path.join(plots_dir, "sched.pdf"), bbox_inches="tight")
plt.close()

# -------------------------
# export data to CSV
# -------------------------

# 1. Save schedulability data (sched.csv)
sched_csv_path = os.path.join(plots_dir, "sched.csv")
with open(sched_csv_path, mode='w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    # Write the header: sweep as x, then the column labels
    writer.writerow([sweep, "PA_schedulability", "RM_schedulability"])
    # Write the data rows
    for row in zip(x, our_sched, their_sched):
        writer.writerow(row)

# 2. Save execution time data (time.csv)
# Note: We use the already scaled data (divided by 1e6) for consistency with the plot
time_csv_path = os.path.join(plots_dir, "time.csv")
with open(time_csv_path, mode='w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    # Write the header
    writer.writerow([
        sweep, 
        "PA_avg_time", "PA_min_time", "PA_max_time", 
        "RM_avg_time", "RM_min_time", "RM_max_time"
    ])
    # Write the data rows
    for row in zip(x, our_avg, our_min, our_max, their_avg, their_min, their_max):
        writer.writerow(row)

print(f"File CSV saved correctly in: {plots_dir}")