export function NoiseChart({ data, labels = false, height = 120 }: { data: number[]; labels?: boolean; height?: number }) {
  const min = 35;
  const max = 92;
  const width = 360;
  const points = data
    .map((value, index) => {
      const x = (index / Math.max(1, data.length - 1)) * width;
      const y = height - ((value - min) / (max - min)) * height;
      return `${x},${Math.max(8, Math.min(height - 8, y))}`;
    })
    .join(" ");

  return (
    <div className="chart-wrap">
      <svg className="noise-chart" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none">
        <rect x="0" y="0" width={width} height={height * 0.34} className="zone zone-red" />
        <rect x="0" y={height * 0.34} width={width} height={height * 0.22} className="zone zone-orange" />
        <rect x="0" y={height * 0.56} width={width} height={height * 0.2} className="zone zone-yellow" />
        <rect x="0" y={height * 0.76} width={width} height={height * 0.24} className="zone zone-green" />
        <polyline points={points} className="chart-line" />
        {data.map((value, index) => {
          if (value < 72) return null;
          const x = (index / Math.max(1, data.length - 1)) * width;
          const y = height - ((value - min) / (max - min)) * height;
          return <circle key={index} cx={x} cy={Math.max(8, Math.min(height - 8, y))} r="3.5" className="chart-peak" />;
        })}
      </svg>
      {labels && (
        <div className="chart-labels">
          <span>Tempo</span>
          <span>Nivel sonoro (dBA)</span>
        </div>
      )}
    </div>
  );
}
