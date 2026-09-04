export type Activity = "Aula" | "Trabalho em grupo" | "Prova" | "Recreio";
export type NoiseStatus = "adequado" | "atencao" | "elevado" | "excessivo";
export type DeviceStatus = "online" | "offline";

export type Room = {
  id: string;
  name: string;
  grade: string;
  activity: Activity;
  noiseLevel: number;
  focusIndex: number;
  status: NoiseStatus;
  peaks: number;
  timeInFocus: number;
  trend: number;
  deviceStatus: DeviceStatus;
  firmware: string;
  updatedAgo: string;
  series: number[];
};

export const activities: Activity[] = ["Aula", "Trabalho em grupo", "Prova", "Recreio"];
export const grades = ["Todas as salas", "9o ano", "8o ano", "7o ano", "6o ano"];

export const activityMeta: Record<Activity, { icon: string; description: string; tone: number }> = {
  Aula: { icon: "📖", description: "Explicacao e instrucao", tone: 55 },
  "Trabalho em grupo": { icon: "👥", description: "Interacao e colaboracao", tone: 65 },
  Prova: { icon: "📝", description: "Concentracao individual", tone: 50 },
  Recreio: { icon: "☕", description: "Nao contabilizado", tone: 78 },
};

export const statusLabels: Record<NoiseStatus, string> = {
  adequado: "Em foco",
  atencao: "Atencao",
  elevado: "Ruido elevado",
  excessivo: "Excessivo",
};

const names = [
  "6A", "6B", "6C", "6D", "6E",
  "7A", "7B", "7C", "7D", "7E",
  "8A", "8B", "8C", "8D", "8E",
  "9A", "9B", "9C", "9D", "9E",
  "Lab Maker", "Biblioteca", "Sala Musica", "Auditorio", "Robótica"
];

const patterns = [-5, -2, 1, 4, 7, 11, 15, 3];

export function getStatus(noiseLevel: number, activity: Activity): NoiseStatus {
  const target = activityMeta[activity].tone;
  if (activity === "Recreio") return noiseLevel <= 84 ? "adequado" : "atencao";
  if (noiseLevel <= target + 3) return "adequado";
  if (noiseLevel <= target + 10) return "atencao";
  if (noiseLevel <= target + 18) return "elevado";
  return "excessivo";
}

export function calculateFocusIndex(noiseLevel: number, activity: Activity, peaks = 3): number {
  if (activity === "Recreio") return 100;
  const target = activityMeta[activity].tone;
  const overage = Math.max(0, noiseLevel - target);
  return Math.max(18, Math.min(98, Math.round(96 - overage * 3.1 - peaks * 1.8)));
}

export function makeSeries(base: number, index: number) {
  return Array.from({ length: 18 }, (_, point) => {
    const wave = Math.sin((point + index) / 2) * 5;
    const pulse = point % 7 === 0 ? (index % 4) * 3 : 0;
    return Math.max(38, Math.round(base + wave + pulse));
  });
}

export const rooms: Room[] = names.map((name, index) => {
  const explicit: Partial<Record<string, { activity: Activity; noiseLevel: number; peaks: number }>> = {
    "9A": { activity: "Aula", noiseLevel: 54, peaks: 4 },
    "9B": { activity: "Trabalho em grupo", noiseLevel: 69, peaks: 5 },
    "8A": { activity: "Prova", noiseLevel: 74, peaks: 7 },
  };
  const activity = explicit[name]?.activity ?? activities[index % activities.length];
  const noiseLevel = explicit[name]?.noiseLevel ?? Math.round(activityMeta[activity].tone + patterns[index % patterns.length]);
  const peaks = explicit[name]?.peaks ?? (index * 3) % 8;
  const status = getStatus(noiseLevel, activity);
  return {
    id: name.toLowerCase().replace(/\s+/g, "-"),
    name,
    grade: name[0]?.match(/[6789]/) ? `${name[0]}o ano` : "Espacos",
    activity,
    noiseLevel,
    focusIndex: name === "9A" ? 87 : calculateFocusIndex(noiseLevel, activity, peaks),
    status,
    peaks,
    timeInFocus: Math.max(32, Math.min(96, 94 - peaks * 4 - Math.max(0, noiseLevel - activityMeta[activity].tone))),
    trend: ((index * 7) % 19) - 5,
    deviceStatus: index === 13 || index === 22 ? "offline" : "online",
    firmware: index % 5 === 0 ? "v1.0.1" : "v1.0.0",
    updatedAgo: index === 13 || index === 22 ? "12 min atras" : `${(index % 8) + 1}s atras`,
    series: makeSeries(noiseLevel, index),
  };
});

export const ranking = [
  { room: "9A", points: 92, highlight: true, delta: 8 },
  { room: "8B", points: 88, highlight: false, delta: 6 },
  { room: "9B", points: 84, highlight: false, delta: 4 },
  { room: "8A", points: 81, highlight: false, delta: 5 },
  { room: "7B", points: 78, highlight: false, delta: 7 },
];

export const weeklyFocus = [
  { label: "Seg", value: 74 },
  { label: "Ter", value: 78 },
  { label: "Qua", value: 81 },
  { label: "Qui", value: 79 },
  { label: "Sex", value: 86 },
];

export const farroupsContribution = [
  { label: "Foco", value: 46 },
  { label: "Recuperacao", value: 28 },
  { label: "Consistencia", value: 31 },
  { label: "Participacao", value: 15 },
];
