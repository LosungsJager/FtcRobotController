import { BarChart3, Gauge, Home, LineChart, Settings, Trophy, Users } from "lucide-react";

const items = [
  { id: "home", label: "Inicio", icon: Home },
  { id: "monitoring", label: "Monitoramento", icon: Gauge },
  { id: "classes", label: "Minhas turmas", icon: Users },
  { id: "ranking", label: "Ranking", icon: Trophy },
  { id: "performance", label: "Desempenho", icon: LineChart },
  { id: "focus", label: "Indice de Foco", icon: BarChart3 },
  { id: "settings", label: "Configuracoes", icon: Settings },
];

export function Sidebar({ active, onNavigate, open, onClose }: { active: string; onNavigate: (page: string) => void; open: boolean; onClose: () => void }) {
  return (
    <>
      <aside className={`sidebar ${open ? "open" : ""}`}>
        <div className="brand" onClick={() => onNavigate("home")} role="button" tabIndex={0}>
          <strong>FARRUÍDO</strong>
          <span>Inteligência acústica</span>
        </div>
        <nav>
          {items.map(({ id, label, icon: Icon }) => (
            <button key={id} className={active === id ? "active" : ""} onClick={() => onNavigate(id)}>
              <Icon size={18} />
              {label}
            </button>
          ))}
        </nav>
        <div className="profile">
          <div className="avatar">H</div>
          <div>
            <strong>Hans</strong>
            <span>Administrador</span>
          </div>
        </div>
      </aside>
      {open && <button className="backdrop" aria-label="Fechar menu" onClick={onClose} />}
    </>
  );
}
