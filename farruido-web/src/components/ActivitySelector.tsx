import { activityMeta, activities, type Activity } from "../services/mockData";

export function ActivitySelector({ value, onChange }: { value: Activity; onChange: (activity: Activity) => void }) {
  return (
    <div className="activity-grid">
      {activities.map((activity) => (
        <button
          key={activity}
          className={`activity-card ${value === activity ? "selected" : ""}`}
          onClick={() => onChange(activity)}
        >
          <span className="activity-icon">{activityMeta[activity].icon}</span>
          <strong>{activity.toUpperCase()}</strong>
          <small>{activityMeta[activity].description}</small>
        </button>
      ))}
    </div>
  );
}
