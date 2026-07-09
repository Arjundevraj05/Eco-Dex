import { getDataFromToken } from "@/helper/getDataFromToken";
import { NextRequest, NextResponse } from "next/server";
import User from "@/models/userModel";
import { connect } from "@/dbConfig/dbConfig";

export async function GET(request: NextRequest) {
    try {
        await connect();
        const userId = await getDataFromToken(request);
        const user = await User.findOne({ _id: userId }).select("-password");

        return NextResponse.json({
            message: "User found",
            data: user
        });
    } catch (error: unknown) { // Use 'unknown' instead of 'any'
        console.error("Error fetching user:", error);

        // Ensure error is an instance of Error before accessing 'message'
        const errorMessage = error instanceof Error ? error.message : "An unknown error occurred";

        return NextResponse.json({ error: errorMessage }, { status: 400 });
    }
}
