#!/bin/bash

# Process all processors in empty_wrappers_verilog folder
for file in empty_wrappers_verilog/*.v; do
  processor_name=$(basename "$file" .v)
  echo "Processing $processor_name..."
  python3 get_automation_coverage.py "$processor_name"
  echo "---"
done

echo ""
echo "Done! Results saved to total.coverage"
