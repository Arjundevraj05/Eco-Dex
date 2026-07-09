import { MongoClient, Db } from "mongodb";
import { isDemoMode } from "@/helper/demoMode";

let client: MongoClient | null = null;
let db: Db | null = null;

export async function connectToDatabase() {
  if (isDemoMode()) {
    throw new Error("Database is disabled in demo mode");
  }

  const uri = process.env.MONGODB_URI as string;
  const dbName = process.env.MONGODB_DB as string;

  if (!uri || !dbName) {
    throw new Error("Please define the MONGODB_URI and MONGODB_DB environment variables");
  }

  if (client && db) {
    return { client, db };
  }

  client = new MongoClient(uri);
  await client.connect();
  db = client.db(dbName);

  console.log("Connected to MongoDB");

  return { client, db };
}
