export interface DemoWasteRecord {
  Class: "PLASTIC" | "METAL" | "PAPER" | "CARDBOARD" | "GLASS";
  isBiodegradable: boolean;
  Latitude: number;
  Longitude: number;
  Day: "Monday" | "Tuesday" | "Wednesday" | "Thursday" | "Friday" | "Saturday" | "Sunday";
}

export const DEMO_WASTE_RECORDS: DemoWasteRecord[] = [
  { Class: "PLASTIC", isBiodegradable: false, Latitude: 12.9716, Longitude: 77.5946, Day: "Monday" },
  { Class: "PAPER", isBiodegradable: true, Latitude: 12.972, Longitude: 77.595, Day: "Monday" },
  { Class: "METAL", isBiodegradable: false, Latitude: 12.973, Longitude: 77.596, Day: "Tuesday" },
  { Class: "CARDBOARD", isBiodegradable: true, Latitude: 12.974, Longitude: 77.597, Day: "Tuesday" },
  { Class: "GLASS", isBiodegradable: false, Latitude: 12.975, Longitude: 77.598, Day: "Wednesday" },
  { Class: "PLASTIC", isBiodegradable: false, Latitude: 12.976, Longitude: 77.599, Day: "Wednesday" },
  { Class: "PAPER", isBiodegradable: true, Latitude: 12.977, Longitude: 77.6, Day: "Thursday" },
  { Class: "METAL", isBiodegradable: false, Latitude: 12.978, Longitude: 77.601, Day: "Thursday" },
  { Class: "CARDBOARD", isBiodegradable: true, Latitude: 12.979, Longitude: 77.602, Day: "Friday" },
  { Class: "GLASS", isBiodegradable: false, Latitude: 12.98, Longitude: 77.603, Day: "Friday" },
  { Class: "PLASTIC", isBiodegradable: false, Latitude: 12.981, Longitude: 77.604, Day: "Saturday" },
  { Class: "PAPER", isBiodegradable: true, Latitude: 12.982, Longitude: 77.605, Day: "Saturday" },
  { Class: "METAL", isBiodegradable: false, Latitude: 12.983, Longitude: 77.606, Day: "Sunday" },
  { Class: "CARDBOARD", isBiodegradable: true, Latitude: 12.984, Longitude: 77.607, Day: "Sunday" },
  { Class: "GLASS", isBiodegradable: false, Latitude: 12.985, Longitude: 77.608, Day: "Monday" },
  { Class: "PLASTIC", isBiodegradable: false, Latitude: 12.986, Longitude: 77.609, Day: "Tuesday" },
  { Class: "PAPER", isBiodegradable: true, Latitude: 12.987, Longitude: 77.61, Day: "Wednesday" },
  { Class: "METAL", isBiodegradable: false, Latitude: 12.988, Longitude: 77.611, Day: "Thursday" },
];

export function getDemoCounts() {
  const counts = DEMO_WASTE_RECORDS.reduce(
    (acc, item) => {
      acc[item.Class] = (acc[item.Class] || 0) + 1;
      return acc;
    },
    { PLASTIC: 0, METAL: 0, PAPER: 0, CARDBOARD: 0, GLASS: 0 }
  );

  const biodegradable = DEMO_WASTE_RECORDS.filter((item) => item.isBiodegradable).length;
  const nonbiodegradable = DEMO_WASTE_RECORDS.length - biodegradable;

  return {
    plastic: counts.PLASTIC,
    paper: counts.PAPER,
    metal: counts.METAL,
    cardboard: counts.CARDBOARD,
    glass: counts.GLASS,
    biodegradable,
    nonbiodegradable,
    totalCount: DEMO_WASTE_RECORDS.length,
  };
}
