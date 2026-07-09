import { NextResponse } from "next/server";
import { withDemoCookies } from "@/helper/demoAuth";
import { isDemoMode } from "@/helper/demoMode";

export async function GET() {
  try {
    const response = NextResponse.json({
      message: isDemoMode() ? "Demo session refreshed" : "Logout successful",
      success: true,
    });

    if (isDemoMode()) {
      return withDemoCookies(response);
    }

    response.cookies.set("auth_token", "", {
      httpOnly: true,
      secure: process.env.NODE_ENV === "production",
      sameSite: "strict",
      expires: new Date(0),
      path: "/",
    });

    response.cookies.set("username", "", {
      httpOnly: true,
      secure: process.env.NODE_ENV === "production",
      sameSite: "strict",
      expires: new Date(0),
      path: "/",
    });

    return response;
  } catch (error: unknown) {
    console.error("Logout error:", error);
    const errorMessage = error instanceof Error ? error.message : "An unknown error occurred";
    return NextResponse.json({ error: errorMessage }, { status: 500 });
  }
}
