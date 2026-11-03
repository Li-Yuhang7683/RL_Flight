// Pure plotting helpers shared by the browser and unit tests.
export function nearestSample(samples, timestamp) {
  if (!samples.length) return null;
  let low = 0, high = samples.length;
  while (low < high) {
    const mid = (low + high) >>> 1;
    if (samples[mid][1] < timestamp) low = mid + 1;
    else high = mid;
  }
  if (low === 0) return samples[0];
  if (low === samples.length) return samples[low - 1];
  return timestamp - samples[low - 1][1] <= samples[low][1] - timestamp
    ? samples[low - 1] : samples[low];
}

export function numericRange(values, fixedMin = null, fixedMax = null) {
  let min = Infinity, max = -Infinity;
  for (const value of values) {
    if (Number.isFinite(value)) { min = Math.min(min, value); max = Math.max(max, value); }
  }
  if (!Number.isFinite(min)) { min = -1; max = 1; }
  const padding = max === min ? Math.max(Math.abs(min) * .05, .1) : (max - min) * .12;
  return [fixedMin ?? min - padding, fixedMax ?? max + padding];
}

export function visibleSegments(samples, start, end, width, gapSeconds = .5) {
  // Preserve gaps and bucket extrema so narrow spikes survive pixel decimation.
  const segments = [];
  let segment = [], bucket = [], pixel = -1, previous = null;
  const flushBucket = () => {
    if (!bucket.length) return;
    let min = bucket[0], max = bucket[0];
    for (const item of bucket) {
      if (item[3] < min[3]) min = item;
      if (item[3] > max[3]) max = item;
    }
    const points = [...new Set([bucket[0], min, max, bucket[bucket.length - 1]])];
    points.sort((a, b) => a[0] - b[0]);
    segment.push(...points);
    bucket = [];
  };
  const flushSegment = () => { flushBucket(); if (segment.length) segments.push(segment); segment = []; pixel = -1; };
  for (const sample of samples) {
    if (sample[1] < start || sample[1] > end) continue;
    if (!Number.isFinite(sample[3])) { flushSegment(); previous = null; continue; }
    if (previous !== null && sample[1] - previous > gapSeconds) flushSegment();
    const nextPixel = Math.floor((sample[1] - start) / Math.max(end - start, .001) * width);
    if (nextPixel !== pixel) { flushBucket(); pixel = nextPixel; }
    bucket.push(sample);
    previous = sample[1];
  }
  flushSegment();
  return segments;
}

export function csvCell(value) {
  const text = value == null ? '' : String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}
