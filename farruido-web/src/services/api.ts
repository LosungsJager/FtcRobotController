import { rooms, ranking, weeklyFocus, farroupsContribution, type Room } from "./mockData";

const wait = (ms = 180) => new Promise((resolve) => setTimeout(resolve, ms));

export async function fetchRooms(): Promise<Room[]> {
  await wait();
  return rooms;
}

export async function fetchRoom(roomId: string): Promise<Room | undefined> {
  await wait(120);
  return rooms.find((room) => room.id === roomId);
}

export async function fetchRanking() {
  await wait(120);
  return ranking;
}

export async function fetchPerformance() {
  await wait(120);
  return { weeklyFocus, farroupsContribution };
}
