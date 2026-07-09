import { NextRequest, NextResponse } from "next/server";
import { getDemoCounts } from "@/helper/demoData";
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
      return NextResponse.json(getDemoCounts());
    }

    const { db } = await connectToDatabase();
    const collection = db.collection(`${auth.username}_waste_records`);

    const [plastic, paper, metal, cardboard, glass, biodegradable, nonbiodegradable, count] =
      await Promise.all([
        collection.countDocuments({ Class: "PLASTIC" }),
        collection.countDocuments({ Class: "PAPER" }),
        collection.countDocuments({ Class: "METAL" }),
        collection.countDocuments({ Class: "CARDBOARD" }),
        collection.countDocuments({ Class: "GLASS" }),
        collection.countDocuments({ isBiodegradable: true }),
        collection.countDocuments({ isBiodegradable: false }),
        collection.countDocuments({}),
      ]);

    return NextResponse.json({
      plastic,
      paper,
      metal,
      cardboard,
      glass,
      biodegradable,
      nonbiodegradable,
      totalCount: count,
    });
  } catch (error) {
    console.error(error);
    return NextResponse.json({ error: "Failed to fetch data" }, { status: 500 });
  }
}
