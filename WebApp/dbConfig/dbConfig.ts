import mongoose from "mongoose";
import { isDemoMode } from "@/helper/demoMode";

let listenersAttached = false;

export async function connect() {
  if (isDemoMode()) {
    return;
  }

  const MONGODB_URI = process.env.MONGODB_URI;

  if (!MONGODB_URI) {
    throw new Error("MONGODB_URI is not defined in environment variables");
  }

  if (mongoose.connection.readyState >= 1) {
    return;
  }

  if (!listenersAttached) {
    mongoose.connection.on("connected", () => {
      console.log("MongoDB Connected");
    });

    mongoose.connection.on("error", (err) => {
      console.error("MongoDB Connection Error:", err);
    });

    listenersAttached = true;
  }

  await mongoose.connect(MONGODB_URI, {
    serverSelectionTimeoutMS: 10000,
  });
}
