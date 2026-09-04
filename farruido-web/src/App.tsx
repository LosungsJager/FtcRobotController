import { useEffect, useMemo, useState } from "react";
import {
  Bell,
  CalendarDays,
  CheckCircle2,
  Lock,
  Menu,
  MonitorCheck,
  School,
  ShieldCheck,
  SignalHigh,
  SlidersHorizontal,
  Sparkles,
  Trophy,
  Users,
} from "lucide-react";
import { ActivitySelector } from "./components/ActivitySelector";
import { DeviceStatus } from "./components/DeviceStatus";
import { FocusIndex } from "./components/FocusIndex";
import { NoiseChart } from "./components/NoiseChart";
import { NoiseStatus } from "./components/NoiseStatus";
import { RoomCard } from "./components/RoomCard";
import { Sidebar } from "./components/Sidebar";
import { StatCard } from "./components/StatCard";
import { fetchPerformance, fetchRanking, fetchRooms } from "./services/api";
import {
  activityMeta,
  activities,
  calculateFocusIndex,
  farroupsContribution,
  getStatus,
  grades,
  makeSeries,
  ranking,
  rooms as fallbackRooms,
  statusLabels,
  weeklyFocus,
  type Activity,
  type Room,
} from "./services/mockData";

type Page = "home" | "monitoring" | "classes" | "ranking" | "performance" | "focus" | "settings" | "room";

const routeFromHash = (): { page: Page; roomId?: string } => {
  const hash = window.location.hash.replace("#/", "");
  if (hash.startsWith("room/")) return { page: "room", roomId: hash.split("/")[1] };
  return { page: (hash || "home") as Page };
};

export function App() {
  const [route, setRoute] = useState(routeFromHash);
  const [rooms, setRooms] = useState<Room[]>([]);
  const [loading, setLoading] = useState(true);
  const [menuOpen, setMenuOpen] = useState(false);
  const [gradeFilter, setGradeFilter] = useState("Todas as salas");
  const [activityFilter, setActivityFilter] = useState<Activity | "Todas">("Todas");
  const [demoLevel, setDemoLevel] = useState(54);
  const [demoActivity, setDemoActivity] = useState<Activity>("Aula");

  useEffect(() => {
    const onHash = () => setRoute(routeFromHash());
    window.addEventListener("hashchange", onHash);
    fetchRooms().then((data) => {
      setRooms(data);
      setLoading(false);
    });
    fetchRanking();
    fetchPerformance();
    return () => window.removeEventListener("hashchange", onHash);
  }, []);

  const data = rooms.length ? rooms : fallbackRooms;
  const currentRoom = data.find((room) => room.id === route.roomId) ?? data.find((room) => room.name === "9A")!;
  const demoStatus = getStatus(demoLevel, demoActivity);
  const demoFocus = calculateFocusIndex(demoLevel, demoActivity, demoLevel > activityMeta[demoActivity].tone + 12 ? 5 : 2);
  const demoSeries = makeSeries(demoLevel, demoLevel % 9);

  const filteredRooms = useMemo(() => {
    return data.filter((room) => {
      const gradeMatch = gradeFilter === "Todas as salas" || room.grade === gradeFilter;
      const activityMatch = activityFilter === "Todas" || room.activity === activityFilter;
      return gradeMatch && activityMatch;
    });
  }, [activityFilter, data, gradeFilter]);

  const navigate = (page: string) => {
    window.location.hash = `/${page}`;
    setMenuOpen(false);
  };

  const openRoom = (id: string) => {
    window.location.hash = `/room/${id}`;
    setMenuOpen(false);
  };

  return (
    <div className="app-shell">
      <Sidebar active={route.page === "room" ? "monitoring" : route.page} onNavigate={navigate} open={menuOpen} onClose={() => setMenuOpen(false)} />
      <main className="main">
        <header className="topbar">
          <button className="icon-btn menu-btn" onClick={() => setMenuOpen(true)} aria-label="Abrir menu">
            <Menu size={20} />
          </button>
          <div className="live-pill"><span /> Dados simulados em tempo real</div>
          <button className="icon-btn" aria-label="Notificacoes"><Bell size={18} /></button>
        </header>
        {loading ? <LoadingState /> : (
          <>
            {route.page === "home" && <Home rooms={data} openRoom={openRoom} />}
            {route.page === "monitoring" && (
              <Monitoring
                rooms={filteredRooms}
                gradeFilter={gradeFilter}
                setGradeFilter={setGradeFilter}
                activityFilter={activityFilter}
                setActivityFilter={setActivityFilter}
                openRoom={openRoom}
              />
            )}
            {route.page === "room" && <RoomDetails room={currentRoom} />}
            {route.page === "classes" && <Classes rooms={data} openRoom={openRoom} />}
            {route.page === "focus" && <FocusPage />}
            {route.page === "ranking" && <RankingPage />}
            {route.page === "performance" && <PerformancePage />}
            {route.page === "settings" && <SettingsPage />}
          </>
        )}
        <DemoMode level={demoLevel} setLevel={setDemoLevel} activity={demoActivity} setActivity={setDemoActivity} status={demoStatus} focus={demoFocus} series={demoSeries} />
      </main>
    </div>
  );
}

function Home({ rooms, openRoom }: { rooms: Room[]; openRoom: (id: string) => void }) {
  const focusRooms = rooms.filter((room) => room.status === "adequado").length;
  const attentionRooms = rooms.filter((room) => room.status === "atencao").length;
  const elevatedRooms = rooms.filter((room) => room.status === "elevado" || room.status === "excessivo").length;
  return (
    <div className="page enter">
      <section className="hero">
        <div>
          <span className="eyebrow">FARRUÍDO</span>
          <h1>Bom dia, Hans</h1>
          <p>Veja como estao os ambientes de aprendizagem hoje.</p>
        </div>
        <div className="hero-device">
          <MonitorCheck size={28} />
          <strong>Inteligência acústica para melhores ambientes de aprendizagem</strong>
          <span>O barulho tambem pode jogar.</span>
        </div>
      </section>
      <div className="stats-grid">
        <StatCard label="Salas monitoradas" value={rooms.length} icon={School} />
        <StatCard label="Salas em foco" value={focusRooms} icon={CheckCircle2} tone="green" />
        <StatCard label="Em atencao" value={attentionRooms} icon={SignalHigh} tone="yellow" />
        <StatCard label="Nivel elevado" value={elevatedRooms} icon={SlidersHorizontal} tone="orange" />
      </div>
      <SectionHeading title="Monitoramento em tempo real" action="Atualiza a cada poucos segundos" />
      <div className="room-grid featured-grid">
        {rooms.slice(15, 18).map((room) => <RoomCard key={room.id} room={room} onOpen={openRoom} />)}
      </div>
      <PrivacyStrip />
    </div>
  );
}

function Monitoring(props: {
  rooms: Room[];
  gradeFilter: string;
  setGradeFilter: (value: string) => void;
  activityFilter: Activity | "Todas";
  setActivityFilter: (value: Activity | "Todas") => void;
  openRoom: (id: string) => void;
}) {
  return (
    <div className="page enter">
      <SectionHeading title="Monitoramento" action={`${props.rooms.length} salas visiveis`} />
      <div className="filters">
        {grades.map((grade) => (
          <button key={grade} className={props.gradeFilter === grade ? "selected" : ""} onClick={() => props.setGradeFilter(grade)}>{grade}</button>
        ))}
      </div>
      <div className="filters compact-filter">
        {(["Todas", ...activities] as const).map((activity) => (
          <button key={activity} className={props.activityFilter === activity ? "selected" : ""} onClick={() => props.setActivityFilter(activity)}>{activity}</button>
        ))}
      </div>
      {props.rooms.length ? (
        <div className="room-grid">
          {props.rooms.map((room) => <RoomCard key={room.id} room={room} onOpen={props.openRoom} />)}
        </div>
      ) : <EmptyState />}
    </div>
  );
}

function RoomDetails({ room }: { room: Room }) {
  return (
    <div className="page enter">
      <section className="detail-hero">
        <div>
          <span className="eyebrow">SALA {room.name}</span>
          <h1>{statusLabels[room.status].toUpperCase()}</h1>
          <p>{activityMeta[room.activity].icon} {room.activity} · {room.noiseLevel} dBA</p>
          <NoiseStatus status={room.status} />
        </div>
        <FocusIndex value={room.focusIndex} size="lg" />
      </section>
      <p className="insight">{room.focusIndex > 80 ? "Excelente ambiente para aprendizagem" : "O ambiente esta se reorganizando para voltar ao melhor equilibrio."}</p>
      <div className="split-grid">
        <section className="panel chart-panel">
          <SectionHeading title="Nivel sonoro ao longo do tempo" action="Modelo contextual experimental" />
          <NoiseChart data={room.series} labels height={220} />
          <div className="legend">
            <span><i className="green" /> adequado</span>
            <span><i className="yellow" /> atencao</span>
            <span><i className="orange" /> elevado</span>
            <span><i className="red" /> picos</span>
          </div>
        </section>
        <DeviceStatus room={room} />
      </div>
      <div className="stats-grid">
        <Metric label="Tempo em nivel adequado" value={`${room.timeInFocus}%`} />
        <Metric label="Picos detectados" value={room.peaks} />
        <Metric label="Maior pico" value={`${Math.max(...room.series)} dBA`} />
        <Metric label="Tempo para recuperacao" value="18 s" />
        <Metric label="Tendencia" value={`${room.trend >= 0 ? "+" : ""}${room.trend}% esta semana`} />
      </div>
      <section className="panel">
        <h2>Qual atividade esta acontecendo?</h2>
        <ActivitySelector value={room.activity} onChange={() => undefined} />
        <p className="note">Os limites de ruido sao adaptados a atividade.</p>
      </section>
    </div>
  );
}

function Classes({ rooms, openRoom }: { rooms: Room[]; openRoom: (id: string) => void }) {
  return (
    <div className="page enter">
      <SectionHeading title="Minhas turmas" action="Visao organizada por ano" />
      <div className="class-list">
        {["9o ano", "8o ano", "7o ano", "6o ano", "Espacos"].map((grade) => {
          const group = rooms.filter((room) => room.grade === grade);
          return (
            <section key={grade} className="panel">
              <SectionHeading title={grade} action={`${group.length} ambientes`} />
              <div className="mini-list">
                {group.map((room) => (
                  <button key={room.id} onClick={() => openRoom(room.id)}>
                    <span>{room.name}</span>
                    <NoiseStatus status={room.status} compact />
                    <b>{room.focusIndex}</b>
                  </button>
                ))}
              </div>
            </section>
          );
        })}
      </div>
    </div>
  );
}

function FocusPage() {
  const components = [
    ["Nivel sonoro", 30],
    ["Duracao do ruido elevado", 18],
    ["Picos", 16],
    ["Recuperacao", 20],
    ["Atividade", 16],
  ];
  return (
    <div className="page enter narrow">
      <section className="detail-hero">
        <div>
          <span className="eyebrow">Indicador experimental</span>
          <h1>Indice de Foco</h1>
          <p>Indice calculado pelo sistema para apoiar decisoes pedagogicas, usando dados simulados nesta demonstracao.</p>
        </div>
        <FocusIndex value={87} size="lg" />
      </section>
      <section className="panel">
        <h2>Metrica composta</h2>
        <div className="component-list">
          {components.map(([label, value]) => (
            <div key={label}>
              <span>{label}</span>
              <div><i style={{ width: `${value}%` }} /></div>
              <b>{value}%</b>
            </div>
          ))}
        </div>
      </section>
      <PrivacyStrip />
    </div>
  );
}

function RankingPage() {
  return (
    <div className="page enter">
      <section className="ranking-hero">
        <div>
          <span className="eyebrow">Competicao saudavel</span>
          <h1>Ranking Farruido</h1>
          <p>Qual turma encontrou o melhor equilibrio?</p>
        </div>
        <div className="tabs"><button className="selected">Esta semana</button><button>Este mes</button><button>Ano</button></div>
      </section>
      <div className="split-grid">
        <section className="panel">
          <div className="ranking-list">
            {ranking.map((item, index) => (
              <div key={item.room} className={item.highlight ? "highlight" : ""}>
                <span>{index === 0 ? "1o" : `${index + 1}o`}</span>
                <strong>{item.room}</strong>
                <b>{item.points} pontos</b>
                <small>Continue melhorando! +{item.delta}%</small>
              </div>
            ))}
          </div>
        </section>
        <BattlePanel />
      </div>
      <section className="panel">
        <SectionHeading title="Evolucao das turmas" action="Ultimos cinco dias" />
        <BarChart data={weeklyFocus} />
      </section>
    </div>
  );
}

function BattlePanel() {
  return (
    <section className="panel battle">
      <Trophy size={28} />
      <span className="eyebrow">Batalha dos Farroups</span>
      <h2>Pontos Farruido conquistados</h2>
      <strong>+120</strong>
      <div className="battle-grid">
        <span>Posicao atual <b>#2</b></span>
        <span>Desempenho da semana <b>+14%</b></span>
      </div>
      <p>O desempenho acustico da turma contribui para a pontuacao da Batalha dos Farroups.</p>
      <BarChart data={farroupsContribution} compact />
    </section>
  );
}

function PerformancePage() {
  return (
    <div className="page enter">
      <SectionHeading title="Desempenho" action="Analise historica" />
      <div className="filters"><button className="selected">Hoje</button><button>7 dias</button><button>30 dias</button><button>Personalizado</button></div>
      <div className="stats-grid">
        <Metric label="Indice medio" value="80" />
        <Metric label="Picos" value="24" />
        <Metric label="Tempo em foco" value="78%" />
        <Metric label="Evolucao" value="+8%" />
      </div>
      <section className="panel">
        <SectionHeading title="Indice de Foco ao longo da semana" action="Seg a Sex" />
        <BarChart data={weeklyFocus} />
      </section>
      <div className="stats-grid">
        <Metric label="Melhor horario" value="08:20" />
        <Metric label="Atividade com melhor desempenho" value="Aula" />
        <Metric label="Maior evolucao" value="9A" />
      </div>
    </div>
  );
}

function SettingsPage() {
  const groups = ["Dispositivo", "Sala", "Rede", "Calibracao", "Notificacoes", "Usuarios"];
  return (
    <div className="page enter">
      <SectionHeading title="Configuracoes" action="Administracao do sistema" />
      <div className="settings-grid">
        {groups.map((group, index) => (
          <section key={group} className="panel setting-card">
            <div className="setting-icon">{index > 2 ? <Lock size={18} /> : <SlidersHorizontal size={18} />}</div>
            <h2>{group}</h2>
            <p>{index > 2 ? "Configuracoes avancadas exigem autenticacao de administrador." : "Parametros operacionais preparados para API real."}</p>
            {index > 2 && <span className="lock-badge">Administrador</span>}
          </section>
        ))}
      </div>
      <PrivacyStrip />
    </div>
  );
}

function DemoMode(props: {
  level: number;
  setLevel: (value: number) => void;
  activity: Activity;
  setActivity: (value: Activity) => void;
  status: ReturnType<typeof getStatus>;
  focus: number;
  series: number[];
}) {
  return (
    <section className="demo">
      <div className="demo-head">
        <Sparkles size={17} />
        <strong>Modo demonstracao</strong>
        <NoiseStatus status={props.status} compact />
      </div>
      <div className="demo-controls">
        <label>
          <span>Nivel sonoro: <b>{props.level} dBA</b></span>
          <input type="range" min="40" max="90" value={props.level} onChange={(event) => props.setLevel(Number(event.target.value))} />
        </label>
        <select value={props.activity} onChange={(event) => props.setActivity(event.target.value as Activity)}>
          {activities.map((activity) => <option key={activity}>{activity}</option>)}
        </select>
        <FocusIndex value={props.focus} size="sm" />
      </div>
      <NoiseChart data={props.series.slice(-12)} height={64} />
    </section>
  );
}

function Metric({ label, value }: { label: string; value: string | number }) {
  return <section className="metric"><span>{label}</span><strong>{value}</strong></section>;
}

function SectionHeading({ title, action }: { title: string; action?: string }) {
  return <div className="section-title"><h2>{title}</h2>{action && <span>{action}</span>}</div>;
}

function BarChart({ data, compact = false }: { data: { label: string; value: number }[]; compact?: boolean }) {
  const max = Math.max(...data.map((item) => item.value));
  return (
    <div className={`bar-chart ${compact ? "compact" : ""}`}>
      {data.map((item) => (
        <div key={item.label}>
          <div><i style={{ height: `${(item.value / max) * 100}%` }} /></div>
          <span>{item.label}</span>
          <b>{item.value}</b>
        </div>
      ))}
    </div>
  );
}

function PrivacyStrip() {
  return (
    <section className="privacy-strip">
      <ShieldCheck size={22} />
      <div>
        <strong>Privacidade em primeiro lugar</strong>
        <span>O Farruido nao grava, armazena ou transmite audio. O sistema processa apenas o nivel sonoro necessario para calcular seus indicadores.</span>
      </div>
    </section>
  );
}

function LoadingState() {
  return <div className="loading"><CalendarDays size={22} /><span>Carregando ambientes monitorados...</span></div>;
}

function EmptyState() {
  return <section className="empty"><School size={30} /><h2>Nenhuma sala encontrada</h2><p>Ajuste os filtros para visualizar outros ambientes.</p></section>;
}
