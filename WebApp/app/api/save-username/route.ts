import { NextRequest, NextResponse } from "next/server";
import { isDemoMode } from "@/helper/demoMode";

export async function POST(request: NextRequest) {
  try {
    if (isDemoMode()) {
      return NextResponse.json({ success: true });
    }

    const { username } = await request.json();

    if (!username) {
      return NextResponse.json({ error: "Username is required" }, { status: 400 });
    }

    const response = NextResponse.json({ success: true });
    response.cookies.set("username", username, { httpOnly: true, maxAge: 24 * 60 * 60 });

    return response;
  } catch (error) {
    console.error("Error saving username:", error);
    return NextResponse.json({ error: "Failed to save username" }, { status: 500 });
  }
}
