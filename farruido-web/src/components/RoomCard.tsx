import { Activity, activityMeta, type Room } from "../services/mockData";
import { NoiseStatus } from "./NoiseStatus";
import { NoiseChart } from "./NoiseChart";

export function RoomCard({ room, onOpen }: { room: Room; onOpen: (id: string) => void }) {
  return (
    <button className="room-card" onClick={() => onOpen(room.id)} aria-label={`Abrir detalhes da sala ${room.name}`}>
      <div className="room-card-head">
        <div>
          <strong>{room.name}</strong>
          <span>{activityMeta[room.activity as Activity].icon} {room.activity}</span>
        </div>
        <NoiseStatus status={room.status} compact />
      </div>
      <div className="room-metric">
        <span>{room.noiseLevel}</span>
        <small>dBA</small>
      </div>
      <NoiseChart data={room.series.slice(-10)} height={72} />
      <div className="room-card-foot">
        <span>Indice de Foco <b>{room.focusIndex}/100</b></span>
        <span>{room.updatedAgo}</span>
      </div>
    </button>
  );
}
