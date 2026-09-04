export function FocusIndex({ value, size = "md" }: { value: number; size?: "sm" | "md" | "lg" }) {
  const radius = 45;
  const circumference = 2 * Math.PI * radius;
  const offset = circumference - (value / 100) * circumference;
  return (
    <div className={`focus focus-${size}`} aria-label={`Indice de Foco ${value} de 100`}>
      <svg viewBox="0 0 120 120" role="img">
        <circle cx="60" cy="60" r={radius} className="focus-track" />
        <circle
          cx="60"
          cy="60"
          r={radius}
          className="focus-progress"
          strokeDasharray={circumference}
          strokeDashoffset={offset}
        />
      </svg>
      <div className="focus-value">
        <strong>{value}</strong>
        <span>/100</span>
      </div>
    </div>
  );
}
