"""Trial-level summaries; failures are reported separately from finite errors."""
import csv
from collections import defaultdict
import numpy as np


def summarize_benchmark(path, bootstrap_samples=2000, seed=0):
    groups = defaultdict(list)
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            key = (float(row['noise_px']), float(row['tangent_noise_deg']), row['refinement'], row['method'])
            groups[key].append(row)
    rng = np.random.default_rng(seed)
    summaries = []
    for (noise, tangent_noise, refinement, method), rows in sorted(groups.items()):
        values = np.array([float(row['error_m']) for row in rows if not row['failure'] and row['error_m']])
        finite = values[np.isfinite(values)]
        interval = [None, None]
        median = None
        if len(finite):
            median = float(np.median(finite))
            medians = np.median(rng.choice(finite, (bootstrap_samples, len(finite)), replace=True), axis=1)
            interval = np.quantile(medians, [0.025, 0.975]).tolist()
        summaries.append(dict(noise_px=noise, tangent_noise_deg=tangent_noise, refinement=refinement,
                              method=method, trials=len(rows), failures=len(rows)-len(finite),
                              median_error_m=median, median_bootstrap_ci95_m=interval,
                              median_elapsed_s=float(np.median([float(row['elapsed_s']) for row in rows]))))
    return summaries


def paired_solver_differences(path, bootstrap_samples=2000, seed=0):
    """Paired 1p5-minus-3p error differences on identical generated trials."""
    trials = defaultdict(dict)
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            key = (float(row['noise_px']), float(row['tangent_noise_deg']),
                   row['refinement'], int(row['trial']), int(row['seed']))
            trials[key][row['method']] = row
    groups = defaultdict(list)
    failures = defaultdict(lambda: {"3p": 0, "1p5": 0, "joint": 0, "total": 0})
    for (noise, tangent_noise, refinement, _, _), methods in trials.items():
        group = (noise, tangent_noise, refinement)
        failures[group]["total"] += 1
        failed3 = "3p" not in methods or bool(methods["3p"]["failure"])
        failed15 = "1p5" not in methods or bool(methods["1p5"]["failure"])
        failures[group]["3p"] += failed3
        failures[group]["1p5"] += failed15
        failures[group]["joint"] += failed3 or failed15
        if not failed3 and not failed15:
            groups[group].append(float(methods["1p5"]["error_m"])-float(methods["3p"]["error_m"]))
    rng = np.random.default_rng(seed)
    output = []
    for group in sorted(failures):
        values = np.asarray(groups[group], float)
        ci = [None, None]
        median = None
        if len(values):
            median = float(np.median(values))
            boot = np.median(rng.choice(values, (bootstrap_samples, len(values)), replace=True), axis=1)
            ci = np.quantile(boot, [0.025, 0.975]).tolist()
        output.append({"noise_px": group[0], "tangent_noise_deg": group[1],
                       "refinement": group[2], "paired_successes": len(values),
                       "trials": failures[group]["total"],
                       "failures_3p": failures[group]["3p"],
                       "failures_1p5": failures[group]["1p5"],
                       "joint_failures": failures[group]["joint"],
                       "median_error_1p5_minus_3p_m": median,
                       "median_difference_bootstrap_ci95_m": ci})
    return output
