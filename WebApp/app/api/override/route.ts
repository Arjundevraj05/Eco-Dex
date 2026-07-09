import { NextRequest, NextResponse } from "next/server";
import { isDemoMode } from "@/helper/demoMode";

export async function POST(req: NextRequest) {
  try {
    const { command } = await req.json();

    if (isDemoMode()) {
      if (["LOCK", "UNLOCK", "ALERT"].includes(command)) {
        return NextResponse.json({
          message: "Command received (demo)",
          status: command,
        });
      }
      return NextResponse.json({ error: "Invalid command" }, { status: 400 });
    }

    const flaskApiUrl = `${process.env.FLASK_BACKEND_URL || "http://localhost:5000"}/api/override`;

    const response = await fetch(flaskApiUrl, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command }),
    });

    const data = await response.json();
    return NextResponse.json(data, { status: response.status });
  } catch (err) {
    console.error("Error sending override command:", err);
    return NextResponse.json({ error: "Failed to send override command" }, { status: 500 });
  }
}
