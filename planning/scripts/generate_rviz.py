import argparse
from pathlib import Path

# --- Argument Parser ---
parser = argparse.ArgumentParser(description="Generate a modular RViz config with grouped displays per drone.")
parser.add_argument('--count', type=int, required=True, help="Number of drones to include")
parser.add_argument('--namespace', type=str, required=True, help="Base name for drone namespaces (e.g. 'iris_velodyne' or 'intelaero_bugwright')")
parser.add_argument('--output', type=str, default="generated.rviz", help="Output RViz file path")
parser.add_argument('--base_template', type=str, required=True)
parser.add_argument('--drone_template', type=str, required=True)
args = parser.parse_args()

# --- Robot Names ---
drone_names = [f"{args.namespace}_{i}" for i in range(args.count)]

# --- Load Base Config ---
with open(args.base_template, "r") as f:
    base_content = f.read()

# --- Load Robot Display Template ---
with open(args.drone_template, "r") as f:
    drone_template = f.read()

# --- Build Robot Display Groups ---
drone_blocks = []
for name in drone_names:
    filled = drone_template.replace("{{DRONE_NAME}}", name)
    drone_blocks.append(filled)

# --- Combine Final Config ---
# Find the insertion point (naive append)
final_content = base_content.rstrip() + "\n"

for block in drone_blocks:
    final_content += block + "\n"

# --- Write Output ---
with open(args.output, "w") as f:
    f.write(final_content)

print(f"[✓] RViz config generated for {args.count} drones (namespace: '{args.namespace}') -> {args.output}")
