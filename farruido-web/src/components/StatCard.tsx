import type { LucideIcon } from "lucide-react";

export function StatCard({ label, value, icon: Icon, tone = "neutral" }: { label: string; value: string | number; icon: LucideIcon; tone?: string }) {
  return (
    <section className={`stat stat-${tone}`}>
      <Icon size={20} />
      <span>{label}</span>
      <strong>{value}</strong>
    </section>
  );
}
