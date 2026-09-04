import { AlertTriangle, CheckCircle2, CircleAlert, SignalHigh } from "lucide-react";
import { statusLabels, type NoiseStatus as Status } from "../services/mockData";

const icons = {
  adequado: CheckCircle2,
  atencao: CircleAlert,
  elevado: SignalHigh,
  excessivo: AlertTriangle,
};

export function NoiseStatus({ status, compact = false }: { status: Status; compact?: boolean }) {
  const Icon = icons[status];
  return (
    <span className={`status status-${status} ${compact ? "compact" : ""}`}>
      <Icon size={compact ? 14 : 16} />
      {statusLabels[status]}
    </span>
  );
}
