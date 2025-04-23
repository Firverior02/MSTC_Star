"""
Analysis and Visualization of Multi-Robot Coverage Algorithm Results

This script loads experimental results for MSTC* and TMSTC* algorithms,
generates publication-ready bar plots with standard error bars,
and produces LaTeX tables summarizing standard errors for each scenario.

Requirements:
    - matplotlib
    - numpy
    - pandas

Input:
    - CSV file: results/results_final.csv

Outputs:
    - SVG figures for each metric/algorithm/robot combination.
    - LaTeX tables (results/standard_error_tables.tex) with standard errors.
"""

from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# --- CONFIGURATION ---

# Input/output file paths
CSV_PATH = "results/results_final.csv"
TEX_OUT_PATH = "results/standard_error_tables.tex"

# Color palettes for each algorithm
MANUAL_COLORS: Dict[str, List[str]] = {
    "MSTC*": ["#D2B4DE", "#BB8FCE", "#9B59B6", "#8E44AD"],
    "TMSTC*": ["#AED6F1", "#5DADE2", "#3498DB", "#2874A6"]
}

# Densities and metrics to consider
DENSITIES = ['5%', '15%', '40%', '50%']
METRICS = ['Time', 'Overlapping']
ALGORITHMS = ['MSTC*', 'TMSTC*']
PAIRS = [("Time", 2), ("Time", 4), ("Overlapping", 2), ("Overlapping", 4)]

# --- UTILITY FUNCTIONS ---

def format_sigfigs(val: float) -> str:
    """Format a number with up to 5 significant digits for table or label display."""
    if pd.isna(val):
        return ""
    elif val == 0:
        return "0"
    elif val >= 10000:
        return f"{val:.0f}"
    elif val >= 1000:
        return f"{val:.1f}"
    elif val >= 100:
        return f"{val:.2f}"
    elif val >= 10:
        return f"{val:.3f}"
    else:
        return f"{val:.4f}"
    
def tex_escape(s: str) -> str:
    """Escape special LaTeX characters in strings (for column/row labels)."""
    return s.replace('%', '\\%').replace('_', '\\_')


# --- MAIN FUNCTIONALITY ---

def plot_metric_final(
    df: pd.DataFrame, 
    metric: str, 
    algorithm: str, 
    num_robots: int, 
    ax: plt.Axes, 
    spacing_cm: float = 0.1, 
    label_offset_cm: float = 1.4
):
    """
    Plot a bar chart for a given metric/algorithm/robot count with error bars.

    Args:
        df: DataFrame with results.
        metric: Column name ('Time' or 'Overlapping').
        algorithm: 'MSTC*' or 'TMSTC*'.
        num_robots: Number of robots (2 or 4).
        ax: Matplotlib Axes to plot on.
        spacing_cm: Spacing between bars, in centimeters.
        label_offset_cm: Offset for value labels above bars, in centimeters.
    """
    # Filter and prepare data
    filtered = df[(df['Algorithm'] == algorithm) & (df['Num_Robots'] == num_robots)]
    grouped = filtered.groupby(['Dimension Label', 'Density Label'])[metric]
    means = grouped.mean().unstack()
    errors = grouped.std().unstack() / np.sqrt(grouped.count().unstack())
    dimensions_ordered = sorted(filtered['Dimension Label'].unique(), key=lambda x: int(x.split()[0]))
    available_densities = [d for d in DENSITIES if d in means.columns]

    # Select/align columns and optionally scale
    means = means.loc[dimensions_ordered, available_densities]
    errors = errors.loc[dimensions_ordered, available_densities]
    if metric == 'Overlapping':
        means *= 100
        errors *= 100

    # Bar plot setup
    colors = MANUAL_COLORS[algorithm]
    x = np.arange(len(means.index))
    bar_width = 0.15
    spacing = spacing_cm / 2.54
    label_offset = label_offset_cm / 2.54
    total_bar_width = len(available_densities) * (bar_width + spacing)
    offset_start = -total_bar_width / 2 + bar_width / 2
    y_offset = 0.016 * (means.values.max() + errors.values.max())

    # Plot each density group
    max_y = 0
    for i, density in enumerate(available_densities):
        offset = offset_start + i * (bar_width + spacing)
        bar_positions = x + offset
        values = means[density].values
        yerr = errors[density].values

        bars = ax.bar(bar_positions, values, width=bar_width,
                      label=density, color=colors[i])

        ax.errorbar(bar_positions, values, yerr=yerr,
                    fmt='none', ecolor='black', elinewidth=1, capsize=5, capthick=1)

        for bar, err in zip(bars, yerr):
            height = bar.get_height()
            label = format_sigfigs(height)
            label_y = height + err + label_offset
            max_y = max(max_y, label_y)
            ax.text(bar.get_x() + bar.get_width() / 2, height + y_offset + err,
                    label, ha='center', va='bottom', fontsize=10, rotation=90)

    # Axes and labels
    ylabel = 'Time (s)' if metric == 'Time' else 'Repeated Coverage Area (%)'
    ax.set_title(f"{ylabel} for {algorithm} with {num_robots} Robots", fontsize=11)
    ax.set_xlabel("Environment Size (cells)")
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)
    ax.set_xticklabels(dimensions_ordered)
    ax.set_ylim(top=max_y * 1.22)
    ax.legend(title="Obstacle Density", loc='upper left' if metric == 'Time' else 'upper right')

    # Reference lines/grid
    ax.set_axisbelow(True)
    ax.grid(True, axis='y', linestyle='-', linewidth=0.6, color='lightgray', alpha=0.8)


def generate_latex_tables(df: pd.DataFrame, pairs, out_path: str):
    """
    Generate LaTeX tables for the standard errors of each metric/algorithm/robots combination.

    Args:
        df: Results DataFrame.
        pairs: List of (metric, robot count) tuples.
        out_path: File path to write LaTeX code.
    """
    latex_output = ""
    for metric, robots in pairs:
        filtered = df[(df['Algorithm'].isin(ALGORITHMS)) & (df['Num_Robots'] == robots)]
        grouped = filtered.groupby(['Algorithm', 'Dimension Label', 'Density Label'])[metric]
        errors = grouped.std().unstack().divide(np.sqrt(grouped.count().unstack()))

        if metric == 'Overlapping':
            errors *= 100

        for algo in ALGORITHMS:
            table = errors.loc[algo]
            # Add missing density columns as NaN for alignment
            for col in DENSITIES:
                if col not in table.columns:
                    table[col] = np.nan
            table = table[DENSITIES]
            table = table.applymap(format_sigfigs)

            # LaTeX code generation
            caption = f"Standard error of {metric.lower()} for {algo} with {robots} robots."
            label = f"tab:{metric.lower()}_{algo.lower().replace('*','star')}_{robots}robots"
            latex_output += "\\begin{table}[H]\n"
            latex_output += f"    \\caption{{{caption}}}\n"
            latex_output += f"    \\vspace{{0.6em}}\n"
            latex_output += f"    \\label{{{label}}}\n"
            latex_output += "    \\centering\n"
            latex_output += "    \\small\n"
            latex_output += "    \\setlength{\\tabcolsep}{10pt}\n"
            latex_output += "    \\renewcommand{\\arraystretch}{1.3}\n"
            latex_output += "    \\begin{tabular}{lcccc}\n"
            latex_output += "        \\hline\n"
            latex_output += "        \\textbf{Environment Size} & \\textbf{5\\%} & \\textbf{15\\%} & \\textbf{40\\%} & \\textbf{50\\%} \\\\\n"
            latex_output += "        \\hline\n"

            for idx, row in table.iterrows():
                row_label = tex_escape(str(idx))
                row_vals = " & ".join(row.values)
                latex_output += f"        {row_label} & {row_vals} \\\\\n"

            latex_output += "        \\hline\n"
            latex_output += "    \\end{tabular}\n"
            latex_output += "\\end{table}\n\n"

    # Write to file
    with open(out_path, "w", encoding='utf-8') as f:
        f.write(latex_output)


# --- SCRIPT ENTRY POINT ---

if __name__ == "__main__":
    # Load CSV and fix headers/labels
    df = pd.read_csv(CSV_PATH, sep=";", decimal=",")
    df['Dimension Label'] = df['Dimension'].astype(str) + " x " + df['Dimension'].astype(str)
    df['Density Label'] = df['Density'].astype(str) + "%"

    # Generate and save all side-by-side plots
    for metric, robots in PAIRS:
        fig, axes = plt.subplots(1, 2, figsize=(10, 5))
        plot_metric_final(df, metric, 'MSTC*', robots, axes[0])
        plot_metric_final(df, metric, 'TMSTC*', robots, axes[1])
        fig.subplots_adjust(left=0.08, right=0.96, top=0.92, bottom=0.28)
        filename = f"results/{metric.replace(' ', '_')}_robots_{robots}.svg"
        fig.savefig(filename, format='svg', bbox_inches='tight')
        plt.close(fig)

    # Generate LaTeX tables for standard errors
    generate_latex_tables(df, PAIRS, TEX_OUT_PATH)