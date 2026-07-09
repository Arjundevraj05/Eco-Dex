import { NextRequest, NextResponse } from "next/server";
import { DEMO_WASTE_RECORDS } from "@/helper/demoData";
import { getAuthFromRequestOrDemo } from "@/helper/getAuthFromRequest";
import { isDemoMode } from "@/helper/demoMode";
import { connectToDatabase } from "@/utils/mongodb";

export async function GET(request: NextRequest) {
  try {
    const auth = getAuthFromRequestOrDemo(request);
    if (!auth) {
      return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
    }

    if (isDemoMode()) {
      return NextResponse.json(DEMO_WASTE_RECORDS);
    }

    const { db } = await connectToDatabase();
    const collection = db.collection(`${auth.username}_waste_records`);
    const documents = await collection.find({}).toArray();

    return NextResponse.json(documents);
  } catch (error) {
    console.error("Error fetching records:", error);
    return NextResponse.json({ error: "Failed to fetch records" }, { status: 500 });
  }
}
