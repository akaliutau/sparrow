#!/bin/bash
set -e

# BATCH_TASK_INDEX is automatically provided by GCP Batch
INPUT_FILE="/mnt/share/inputs/config_${BATCH_TASK_INDEX}.json"
OUTPUT_FILE="/mnt/share/outputs/result_${BATCH_TASK_INDEX}.json"

# Use Environment Variables or Defaults
# SEED default: 23, EPOCHS default: 200, CANDIDATES default: 200
SEED_VAL=${SEED:-23}
EXPLORE_VAL=${EXPLORE:-300}
COMPRESS_VAL=${COMPRESS:-200}

echo "Processing Task ${BATCH_TASK_INDEX}"
echo "Params: Seed=${SEED_VAL}, Epochs=${EXPLORE_VAL}, Candidates=${COMPRESS_VAL}"

# Run the binary with dynamic flags
/app/sparrow -i "${INPUT_FILE}" -s "${SEED_VAL}" -e "${EXPLORE_VAL}" -c "${COMPRESS_VAL}"
# Will fail if multiple files exist because cp expects 2 arguments (source, dest)
cp /app/output/*.json "$OUTPUT_FILE" 2>/dev/null || echo "Error: Copy failed (0 or >1 files found)"

echo "Done. Result saved to ${OUTPUT_FILE}"
