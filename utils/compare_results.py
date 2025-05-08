"""
CSV File Row Comparison Utility

This script compares the content of two CSV files, ignoring row order,
and reports any differences between them. Each row is compared as a tuple
of string values. This is useful for verifying that two result files are
identical in content, regardless of the row sequence.

Assumes the CSV files use a semicolon (;) as the delimiter and have a header row.

Usage Example:
    compare_csv_files("results1.csv", "results2.csv")
"""

import csv
from typing import Set, Tuple


def load_csv_as_set(path: str) -> Set[Tuple[str, ...]]:
    """
    Load a CSV file and return its data rows as a set of tuples (excluding the header).

    Args:
        path: Path to the CSV file.

    Returns:
        Set of rows, each represented as a tuple of string values.
    """
    with open(path, newline='', encoding='utf-8') as file:
        reader = csv.reader(file, delimiter=';')
        next(reader, None)  # Skip header
        rows = {tuple(row) for row in reader}
    return rows


def compare_csv_files(file1: str, file2: str) -> None:
    """
    Compare two CSV files and print a report of any differing rows.

    Args:
        file1: Path to the first CSV file.
        file2: Path to the second CSV file.
    """
    set1 = load_csv_as_set(file1)
    set2 = load_csv_as_set(file2)

    if set1 == set2:
        print("The files contain the same rows.")
    else:
        only_in_1 = set1 - set2
        only_in_2 = set2 - set1

        print("The files differ.")
        if only_in_1:
            print(f"\nRows only in '{file1}':")
            for row in sorted(only_in_1):
                print("   ", row)
        if only_in_2:
            print(f"\nRows only in '{file2}':")
            for row in sorted(only_in_2):
                print("   ", row)


if __name__ == "__main__":
    # Example usage
    compare_csv_files("results1.csv", "results2.csv")
