require("dotenv").config({ path: ".env.local" });
require("dotenv").config({ path: ".env" });

const { MongoClient } = require("mongodb");
const mongoose = require("mongoose");

async function main() {
  const uri = process.env.MONGODB_URI;
  const dbName = process.env.MONGODB_DB || "ecodex";

  if (!uri) {
    console.error("MONGODB_URI is not set. Copy .env.example to .env.local and fill it in.");
    process.exit(1);
  }

  console.log("Testing MongoDB connection...");
  console.log(`Database name: ${dbName}`);

  const client = new MongoClient(uri);
  await client.connect();
  const db = client.db(dbName);
  await db.command({ ping: 1 });
  console.log("Native driver: OK");

  await mongoose.connect(uri);
  console.log("Mongoose: OK");

  await mongoose.disconnect();
  await client.close();

  console.log("\nConnection successful. You can run: npm run dev");
}

main().catch((error) => {
  console.error("\nConnection failed:", error.message);
  console.error("\nChecklist:");
  console.error("- MONGODB_URI is correct (password URL-encoded if it has special chars)");
  console.error("- Your IP is allowed in Atlas Network Access");
  console.error("- Database user exists in Atlas Database Access");
  process.exit(1);
});
