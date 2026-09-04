import { Wifi, Cpu, RadioTower } from "lucide-react";
import type { Room } from "../services/mockData";

export function DeviceStatus({ room }: { room: Room }) {
  const online = room.deviceStatus === "online";
  return (
    <section className="device-panel">
      <div className="section-title">
        <div>
          <span className="eyebrow">FARRUÍDO DEVICE</span>
          <h3>{online ? "ONLINE" : "OFFLINE"}</h3>
        </div>
        <span className={`device-dot ${online ? "online" : "offline"}`} />
      </div>
      <div className="device-grid">
        <span>Sala <b>{room.name}</b></span>
        <span>Firmware <b>{room.firmware}</b></span>
        <span>Ultima atualizacao <b>{room.updatedAgo}</b></span>
        <span><Cpu size={14} /> Sensor <b>{online ? "conectado" : "sem sinal"}</b></span>
        <span><Wifi size={14} /> Wi-Fi <b>{online ? "conectado" : "instavel"}</b></span>
        <span><RadioTower size={14} /> RS485 <b>{online ? "ativo" : "aguardando"}</b></span>
      </div>
    </section>
  );
}
